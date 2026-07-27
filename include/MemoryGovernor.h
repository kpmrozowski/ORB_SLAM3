/**
 * MemoryGovernor — env-gated periodic sampler of process memory and Atlas keyframe/map-point
 * bookkeeping, plus (Task P1+) the allocator-hygiene knobs for the memory-reduction project.
 *
 * ORB_MEM_STATS_CSV=<path>   enable sampling; one CSV row is appended to <path> every
 *                            ORB_MEM_STATS_EVERY processed frames. Unset => AppendStats() costs
 *                            a single static-bool check (zero per-frame overhead).
 * ORB_MEM_STATS_EVERY=<n>    sampling period in processed frames (default 30).
 * ORB_MEM_RECLAIM_BAD=1      (Task P1) enable KeyFrame::ReleaseBadPayload()/MapPoint descriptor
 *                            release, M_ARENA_MAX and the malloc_trim cadence in Tick().
 *                            Deterministic-mode-only (env ORB_DETERMINISTIC): requesting it
 *                            without deterministic mode prints one warning line and stays inert
 *                            (SetBadFlag() is reachable from LocalMapping/LoopClosing background
 *                            threads outside deterministic mode; ReclaimBadPayloadEnabled() must
 *                            therefore be safe to call from any thread — see .cc).
 *
 *                            KeyFrame payload release is INLINE (Task P1-fix): KeyFrame::
 *                            SetBadFlag() calls ReleaseBadPayload() on itself directly, at the
 *                            very end of the function, before returning. The original design
 *                            deferred it by one KF-tick via a DeferKeyFrameRelease() queue that
 *                            Tick() drained, on the assumption that KeyFrames are never
 *                            delete()d (so a queued pointer could never dangle) — that assumption
 *                            is FALSE: LocalMapping::InitializeIMU() and ::ScaleRefinement() both
 *                            run `(*lit)->SetBadFlag(); delete *lit;` back to back on entries of
 *                            mlNewKeyFrames, reachable every frame from SpinOnceDeterministic().
 *                            The deferred design therefore queued a pointer that could be freed
 *                            before Tick() dereferenced it — a use-after-free (see
 *                            task-P1-fix-report.md for the reproduction). Inline release closes
 *                            the window: nothing can delete `this` between SetBadFlag() enqueuing
 *                            it and the release actually running, because there is no longer any
 *                            gap between the two. This is safe for the fields KeyFrame::
 *                            ReleaseBadPayload() actually frees (mDescriptors/mBowVec/mFeatVec/
 *                            mGrid/mvKeys/mvDepth): every reader of them is either isBad()-guarded
 *                            or unreachable in monocular mode, verified by the required 4-cell
 *                            determinism gate. mvKeysUnData/mvuRight/mvpMapPoints are NOT released
 *                            (KeyFrame::ReleaseBadPayload() keeps them, unchanged from Task P1)
 *                            precisely because of the opposite problem — stale observations from
 *                            other, live MapPoints make load-bearing unguarded reads into them.
 *                            Task P3a's accessor refactor routes every external read of these
 *                            three through fault-in choke points (KeyFrame::GetKeysUn()/
 *                            GetKpURight()/GetMapPoint() etc.) and investigated actually releasing
 *                            them (adding the missing isBad() guard the choke points made
 *                            possible to add) — that guard was empirically proven, via this
 *                            task's own required gate, to change the OFF-mode trajectory on the
 *                            gate flight, so it was not shipped; see task-P3a-report.md.
 *
 *                            MapPoint descriptor release REMAINS DEFERRED BY ONE KF-TICK: unlike
 *                            KeyFrames, MapPoints that ever reach SetBadFlag() are never
 *                            delete()d anywhere in this codebase (verified by grep — see
 *                            task-P1-fix-report.md), so there is no dangling-pointer risk to fix
 *                            for them. But stock tracking legitimately reads a just-culled
 *                            MapPoint's descriptor during the very next frame —
 *                            Tracking::TrackWithMotionModel matches against bad MapPoints'
 *                            descriptors carried in mLastFrame.mvpMapPoints via
 *                            ORBmatcher::SearchByProjection (SearchLocalPoints only scrubs them
 *                            from the chain at that frame's TrackLocalMap) — so immediate release
 *                            of the descriptor would race that legitimate read. This was proven
 *                            empirically: immediate release produced a reproducible SIGSEGV in
 *                            ORBmatcher::DescriptorDistance (null-data read of a released
 *                            cv::Mat), independently re-derived by reading the exact call chain
 *                            (task-P1-report.md, Finding 1 §5). MapPoint::SetBadFlag() therefore
 *                            still enqueues via DeferMapPointRelease(); Tick() — which runs after
 *                            the LocalMapping/LoopClosing spins of every tracked frame — releases
 *                            MapPoints enqueued before the previous tick, i.e. only after the one
 *                            frame that may still read them stock-legitimately has completed.
 * ORB_MEM_PARANOIA=1         (Task P1, validation-only) poison released KeyFrame payload fields
 *                            and abort-with-object-id on any read through KeyFrame::ComputeBoW()/
 *                            KeyFrame::GetFeaturesInArea()/MapPoint::GetDescriptor() after
 *                            release. See task-P1-report.md for exact coverage (partial by
 *                            design; full coverage arrives with P3a's accessors).
 *
 * P0 scope was measurement only: no eviction, no spill (Delta RSS = 0). Task P1 starts actually
 * freeing memory; the `kf_hot`, `kf_cold`, `spill_mb` and `faultins_total` CSV columns remain
 * reserved for later phases and always read 0 here (the header is written once, complete and
 * stable, so downstream CSV consumers do not break when those phases populate the columns) —
 * `kf_shell` is now live, fed by IncrementKfShellReleased().
 *
 * IMPORTANT for maintainers extending AppendStats() (or anything else on a per-frame/per-tick
 * hot path): this codebase's deterministic-mode floating-point output is measurably sensitive to
 * the process's heap allocation *pattern* (bisected empirically — see MemoryGovernor.cc's
 * ReadProcStatusFieldKb/WriteAll comments and the P0 report). A *persistent* allocation added
 * anywhere on the sampled-frame hot path (e.g. a std::ofstream/std::string/std::vector that
 * outlives the call, unlike the transient vectors Atlas::GetAllMaps()/GetAllKeyFrames()/
 * GetAllMapPoints() already return) is enough to change the trajectory bit-for-bit versus stats
 * disabled, even though nothing about the SLAM inputs changed. The implementation therefore uses
 * only raw POSIX fd I/O and fixed-size stack buffers on that path — keep it that way (or
 * re-verify the fast-gate md5 gate if you don't). Task P0.5 additionally made the pipeline
 * allocation-*invariant* (no result-affecting reads of never-written heap memory survive), which
 * is what makes Task P1's actual freeing safe in the first place — freeing changes the
 * allocation pattern by design, but no longer changes the computed trajectory, PROVIDED the
 * freed objects are truly unreachable for future tracking/mapping decisions (verified per-reader
 * in task-P1-report.md) and this governor's own bookkeeping stays allocation-free in steady
 * state (std::atomic counters; the deferred-release lists are reserve()d once when reclaim is
 * active and only clear()ed/swap()ed afterwards — capacity is retained, so the per-tick path
 * performs no heap allocation once warmed up).
 */
#ifndef MEMORYGOVERNOR_H
#define MEMORYGOVERNOR_H

#include <atomic>
#include <deque>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ORB_SLAM3
{

class Atlas;
class MapPoint;
class KeyFrame;
class SpillWorker;

class MemoryGovernor
{
public:
    static MemoryGovernor& Instance();

    MemoryGovernor(const MemoryGovernor&) = delete;
    MemoryGovernor& operator=(const MemoryGovernor&) = delete;

    // Non-owning hook: called once from System::System() so AppendStats() can read live
    // KF/MP/map counts through the Atlas. The governor never outlives the Atlas (both live for
    // the process lifetime once constructed), and every Atlas read inside AppendStats() is
    // guarded on this pointer being non-null, so calling AppendStats() before SetAtlas() (or
    // never calling SetAtlas() at all, e.g. outside deterministic mode) is safe.
    void SetAtlas(Atlas* const atlas);

    // Appends one CSV row to ORB_MEM_STATS_CSV every ORB_MEM_STATS_EVERY processed frames
    // (default 30); a no-op costing a single static-bool check when ORB_MEM_STATS_CSV is unset.
    void AppendStats(const double timestamp_seconds);

    // Task P3d-evict — hand the governor the CURRENT tracking working set (Tracking::
    // mvpLocalKeyFrames + reference KeyFrame) so the next EvictionSweep() protects it, plus the
    // reference KeyFrame's direct covisibles, from spill eviction — not just the last-W-created
    // window. Without this the active local/covisibility window is evicted and re-faulted every
    // frame (the P3d thrash: ~160 fault-ins/frame). Call once per tracked frame from
    // System::TrackMonocular(), immediately before Tick(), on the deterministic main thread. Stores
    // only non-owning observers into Tracking's live state (see the members); a cheap store when
    // spill is inactive (EvictionSweep never runs then). The referenced vector need only outlive the
    // immediately-following Tick() — it does: it is Tracking's live member, same thread, unmutated
    // in between.
    void SetTrackingWorkingSet(const std::vector<KeyFrame*>& local_key_frames,
                               KeyFrame* const reference_key_frame);

    // Per-KF-tick hook (Task P1): call once per tracked frame, after the deterministic
    // LocalMapping/LoopClosing spin (System::TrackMonocular). Releases the descriptor of
    // MapPoints enqueued before the previous tick (see the ORB_MEM_RECLAIM_BAD deferral note in
    // the file header) and runs malloc_trim(0) every 100 calls — only when
    // ReclaimBadPayloadEnabled(); otherwise a single static-bool check. KeyFrame payload release
    // is no longer part of Tick() (Task P1-fix): it now runs inline from KeyFrame::SetBadFlag().
    void Tick();

    // Enqueue a MapPoint whose SetBadFlag() just completed for descriptor release at the
    // next-plus-one Tick(). Only ever called (and only meaningful) when
    // ReclaimBadPayloadEnabled(); deterministic single-thread only. MapPoints registered in a Map
    // are never deleted (the only `delete pMP` in the codebase is for stereo-only temporal
    // points, which are never SetBadFlag()'d), so queued pointers cannot dangle. (KeyFrames do
    // not get this treatment as of Task P1-fix — see the file header — because that assumption
    // does NOT hold for KeyFrames: LocalMapping::InitializeIMU()/::ScaleRefinement() do delete
    // them right after SetBadFlag(), which is what made the deferred KeyFrame queue a
    // use-after-free.)
    void DeferMapPointRelease(MapPoint* const map_point);

    // Task P6 — quarantine delete of culled MapPoints (ORB_MEM_DELETE_QUARANTINE=K KF-ticks).
    //
    // Enqueue a MapPoint whose SetBadFlag()/Replace() just completed (so it is unlinked from the
    // Map and every observer KeyFrame slot) for actual delete() after a K-keyframe quarantine.
    // Only ever called (and only meaningful) when DeleteQuarantineActive(); deterministic
    // single-thread only, so the queue needs no locking. MapPoint::mbDeleteQueued guarantees each
    // MapPoint is enqueued at most once, so it is delete()d at most once (the single sanctioned
    // owning free). The pointer/container coverage proof for why the quarantine makes this
    // use-after-free-safe is in task-P6-report.md; the short version:
    //   * Map::mspMapPoints           -- erased synchronously in SetBadFlag()/Replace().
    //   * live KeyFrame mvpMapPoints  -- every slot nulled synchronously via EraseMapPointMatch()
    //                                    over the (complete-for-live-holders) mObservations set.
    //   * bad/shell KeyFrame slots    -- never dereferenced (bad KFs are unlinked from the Map,
    //                                    the KeyFrameDatabase and the covisibility graph, so no
    //                                    reader iterates their mvpMapPoints).
    //   * Frame::mvpMapPoints (last/current) + MapPoint::mpReplaced chain (CheckReplacedInLastFrame)
    //                                 -- transient, cleared within ~1-2 frames << K keyframes.
    //   * LocalMapping::mlpRecentAddedMapPoints -- a bad MapPoint is erased on the next
    //                                    MapPointCulling pass, i.e. within <=3 keyframes; K>=5
    //                                    covers it. This is why K is counted in KEYFRAMES.
    //   * LoopClosing scratch vectors -- re-populated from live KFs each detection tick and
    //                                    consumed within the same cycle.
    void DeferMapPointDelete(MapPoint* const map_point);

    // True iff ORB_MEM_DELETE_QUARANTINE>0 AND ORB_DETERMINISTIC is active. Gates the
    // SetBadFlag()/Replace() enqueues and the delete pass in Tick(). Requesting it without
    // deterministic mode prints exactly one warning and stays inert (the quarantine's
    // use-after-free safety proof relies on the single-threaded deterministic tick point; frees
    // in threaded mode could race stale readers). K=0/unset => fully inert = stock leak behaviour.
    // Callable from any thread/file; caches its result in function-local statics like the peers.
    static bool DeleteQuarantineActive();

    // Current resident set size in KB, read from /proc/self/status VmRSS. 0 if unavailable.
    static long ReadVmRssKb();

    // Peak resident set size in KB, read from /proc/self/status VmHWM. 0 if unavailable.
    static long ReadVmHwmKb();

    // True iff ORB_MEM_RECLAIM_BAD is requested AND ORB_DETERMINISTIC is active. Reads both env
    // vars once (function-local statics, thread-safe per C++11 magic-statics); if requested
    // without deterministic mode, prints exactly one warning line (across the whole process,
    // regardless of how many threads observe it first) and returns false. Callable from any
    // thread/any file; gates the SetBadFlag() enqueues, ConfigureAllocatorIfEnabled() and
    // Tick()'s release/trim work.
    static bool ReclaimBadPayloadEnabled();

    // True iff ORB_MEM_PARANOIA=1. Validation-only: makes ReleaseBadPayload() poison released
    // fields with recognizable sentinels and makes KeyFrame::ComputeBoW()/GetFeaturesInArea()
    // and MapPoint::GetDescriptor() abort (with the object id) if called after release. No
    // effect by itself unless ReclaimBadPayloadEnabled() is also true (poisoning only ever
    // happens inside the release methods).
    static bool ParanoiaEnabled();

    // Allocator hygiene (Task P1): mallopt(M_ARENA_MAX, 2), meant to be called once from
    // System::System() ("at startup", before KeyFrame/MapPoint payload allocation ramps up). A
    // no-op unless ReclaimBadPayloadEnabled(), and a no-op on non-glibc allocators.
    static void ConfigureAllocatorIfEnabled();

    // Bumped once per KeyFrame whose payload was actually released by
    // KeyFrame::ReleaseBadPayload(); feeds AppendStats()'s kf_shell column. Atomic, lock-free, no
    // allocation — safe to call from any thread even though reclaim itself only ever runs on the
    // single deterministic-mode thread (ReclaimBadPayloadEnabled() ensures that).
    static void IncrementKfShellReleased();

    // Task P3d — cold-KF payload spill + eviction (ORB_MEM_BUDGET_MB).
    //
    // True iff ORB_MEM_BUDGET_MB>0 AND ORB_DETERMINISTIC is active. Master enable for the whole
    // spill subsystem (worker thread, eviction sweep, fault-in). Requesting it in threaded mode
    // prints one warning and stays inert (frees can only be made race-free at the single
    // deterministic tick — see plan decision 5). ORB_MEM_BUDGET_MB=0/unset => fully inert = stock.
    static bool SpillActive();

    // Fault-in choke, called from KeyFrame::EnsureResident() ONLY on a cold miss (the hot-path
    // acquire-load already returned). Blocks THIS caller until the payload is resident; lazily
    // starts the SpillWorker. Updates the KeyFrame's LRU tick.
    void FaultIn(KeyFrame* const key_frame);

    // Read-ahead hook: queue a loop/reloc/merge candidate (and its covisibles) for background
    // page-in after BoW candidate selection, before geometric matching touches its payload. No-op
    // unless SpillActive() and the KeyFrame is currently evicted.
    void Prefetch(KeyFrame* const key_frame);

    // Join the background SpillWorker. Called from System::Shutdown(); idempotent.
    void ShutdownSpill();

    // Current governor spill tick (monotonic, bumped once per eviction sweep). Used as the LRU
    // stamp written into KeyFrame::mSpillLastUseTick on fault-in.
    long CurrentSpillTick() const { return mSpillTick.load(std::memory_order_relaxed); }

private:
    MemoryGovernor() = default;

    // Task P3d helpers (all main-thread-only, called from Tick()/FaultIn()).
    void EnsureSpillWorker();
    void EvictionSweep();

    // Task P3d-evict — rebuild mProtectScratch for this tick from the working set handed in by
    // SetTrackingWorkingSet(): { Tracking::mvpLocalKeyFrames } ∪ { reference KeyFrame } ∪ { its
    // direct covisibles }. The last-W-created window is handled separately by the mnId check in
    // EvictionSweep() and is not duplicated here. Read-only over live state; reuses the set's
    // capacity across ticks (allocation-invariant per P0.5). Called once per EvictionSweep().
    void PopulateProtectScratch();

    // Task P6 — move MapPoints whose K-keyframe quarantine has expired out of the FIFO
    // mDeleteQueue into the pending-scan batch, then (amortized: once every
    // ORB_MEM_DELETE_SCAN_EVERY ticks, or immediately once the batch exceeds
    // ORB_MEM_DELETE_SCAN_CAP) run the slot-severance scan + batch free via
    // SeverAndDeleteBatch(). Called from Tick() (deterministic main thread, after the
    // LocalMapping/LoopClosing spins). Detects a full Tracking::Reset (KeyFrame::nNextId went
    // backwards, atlas cleared) and drops both the queue and the batch without freeing (those
    // MapPoints are orphaned by the reset; leaking the last <K keyframes' worth matches stock and
    // avoids a post-reset UAF).
    void DrainExpiredDeletes();

    // Task P6 (use-after-free fix) — the sanctioned batch free. BEFORE delete()ing the batch, scan
    // every live keyframe (all maps in the atlas; GetAllKeyFrames() excludes bad/shell KFs) and
    // null any mvpMapPoints slot still pointing at a batched MapPoint. ORB-SLAM3 deliberately keeps
    // mvpMapPoints slots desynchronized from MapPoint::mObservations, so SetBadFlag()/Replace()
    // severance (which walks mObservations) can leave a stale live-keyframe slot dangling; this
    // pass removes it, guaranteeing no live-and-readable container holds the pointer at free time.
    // Shell-keyframe slots are made unreadable instead by the isBad() guards in
    // Tracking::UpdateLocalKeyFrames() (shells are excluded from GetAllKeyFrames()). Amortized: the
    // O(keyframes * features) scan runs once per batch (see DrainExpiredDeletes()), not per tick,
    // with O(1) membership via mDoomedScratch — negligible per-frame CPU.
    void SeverAndDeleteBatch();

    Atlas* mpAtlas = nullptr;
    std::atomic<long> mKfShellReleased{0};

    std::unique_ptr<SpillWorker> mSpillWorker;
    bool mSpillDisabled = false;                 // set if the spill file could not be opened
    std::atomic<long> mSpillTick{0};
    std::vector<KeyFrame*> mEvictCandidatesScratch;  // reused across ticks (no per-tick alloc)

    // Task P3d-evict — current tracking working set, handed in by SetTrackingWorkingSet() right
    // before each Tick() and consumed synchronously in the same-thread EvictionSweep() that
    // immediately follows. Non-owning observers into Tracking's live state; refreshed every
    // deterministic frame and never read across ticks, so they cannot dangle. nullptr until the
    // first frame (and always, outside deterministic mode) -- EvictionSweep()/PopulateProtectScratch()
    // guard on the pointer.
    const std::vector<KeyFrame*>* mWorkingSetLocalKeyFrames = nullptr;
    KeyFrame* mWorkingSetReferenceKeyFrame = nullptr;
    // Per-tick eviction protect set (the local window + reference covisibles), rebuilt each sweep by
    // PopulateProtectScratch(); reused across ticks (capacity retained via clear()).
    std::unordered_set<KeyFrame*> mProtectScratch;

    // Two-phase deferred-release queue for MapPoints (see the ORB_MEM_RECLAIM_BAD header note):
    // MapPoints enqueued at tick T sit in mPendingMapPointRelease, move to
    // mReadyMapPointRelease at Tick(T), and are released at Tick(T+1) — strictly after the one
    // frame that may still read them stock-legitimately. reserve()d once on first enqueue;
    // clear() (capacity-retaining) + swap thereafter, so the steady-state per-tick path allocates
    // nothing. KeyFrames no longer go through a queue at all (Task P1-fix): their release is
    // inline from KeyFrame::SetBadFlag().
    std::vector<MapPoint*> mPendingMapPointRelease;
    std::vector<MapPoint*> mReadyMapPointRelease;

    // Task P6 quarantine delete queue. Each entry is (culled MapPoint, KeyFrame::nNextId at the
    // moment it was baddened). Entries are appended in baddening order, and KeyFrame::nNextId is
    // monotonic within a reset epoch, so the queue is intrinsically FIFO-ordered by that id ->
    // DrainExpiredDeletes() only ever pops from the front. Touched exclusively on the single
    // deterministic thread (enqueue from SetBadFlag()/Replace() during the spins, drain from
    // Tick()), so it is lock-free by construction. std::deque node churn is bounded (only the
    // last K keyframes' worth of culled MapPoints are ever resident) and, post-P0.5, allocation
    // is trajectory-invariant, so this does not perturb the deterministic md5.
    std::deque<std::pair<MapPoint*, long>> mDeleteQueue;
    // Last KeyFrame::nNextId observed by DrainExpiredDeletes(); a decrease flags a full reset.
    long mLastKeyFrameNextIdSeen = 0;

    // Task P6 (use-after-free fix) — amortized slot-severance batch. Quarantine-expired MapPoints
    // are moved here from mDeleteQueue but NOT freed until SeverAndDeleteBatch() has nulled every
    // live-keyframe slot pointing at them; the batch is flushed once per kScanEveryTicks ticks (or
    // sooner if it exceeds kScanBatchCap), amortizing the atlas scan so per-tick CPU stays
    // negligible. A quarantine-expired MapPoint held here for the extra <kScanEveryTicks ticks is
    // simply quarantined a little longer -- strictly safer, and md5-neutral (the trajectory is
    // allocation-invariant post-P0.5, and nulling a stale slot equals stock's lazy null-on-read).
    // mDoomedScratch is the reused O(1)-membership set (capacity retained across flushes).
    std::vector<MapPoint*> mPendingDeleteScan;
    std::unordered_set<MapPoint*> mDoomedScratch;
    long mDeleteScanTick = 0;
};

}  // namespace ORB_SLAM3

#endif  // MEMORYGOVERNOR_H
