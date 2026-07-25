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
 *                            Release is DEFERRED BY ONE KF-TICK, not immediate: stock tracking
 *                            legitimately reads a just-culled object's payload during the very
 *                            next frame — TrackWithMotionModel matches against bad MapPoints'
 *                            descriptors carried in mLastFrame.mvpMapPoints (SearchLocalPoints
 *                            only scrubs them from the chain at that frame's TrackLocalMap),
 *                            and TrackReferenceKeyFrame/NeedNewKeyFrame can read a just-culled
 *                            mpReferenceKF's mFeatVec/mvpMapPoints before UpdateLocalKeyFrames
 *                            reassigns it. Immediate release at SetBadFlag() therefore crashes
 *                            (proven: SIGSEGV in ORBmatcher::DescriptorDistance via
 *                            mLastFrame.mvpMapPoints 15s into the fast gate) or silently
 *                            diverges the md5. SetBadFlag() instead enqueues via
 *                            DeferKeyFrameRelease()/DeferMapPointRelease(); Tick() — which runs
 *                            after the LocalMapping/LoopClosing spins of every tracked frame —
 *                            frees objects enqueued before the previous tick, i.e. only after
 *                            the one frame that may still read them stock-legitimately has
 *                            completed. By then every other reachability path is already severed
 *                            inside SetBadFlag() (Map/KFDB erasure, observation severance, chain
 *                            scrub, reference reassignment), verified per-reader in
 *                            task-P1-report.md.
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
#include <vector>

namespace ORB_SLAM3
{

class Atlas;
class KeyFrame;
class MapPoint;

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

    // Per-KF-tick hook (Task P1): call once per tracked frame, after the deterministic
    // LocalMapping/LoopClosing spin (System::TrackMonocular). Releases the payload of KeyFrames/
    // MapPoints enqueued before the previous tick (see the ORB_MEM_RECLAIM_BAD deferral note in
    // the file header) and runs malloc_trim(0) every 100 calls — only when
    // ReclaimBadPayloadEnabled(); otherwise a single static-bool check.
    void Tick();

    // Enqueue a KeyFrame whose SetBadFlag() just completed for payload release at the
    // next-plus-one Tick(). Only ever called (and only meaningful) when
    // ReclaimBadPayloadEnabled(); deterministic single-thread only. KeyFrames are never deleted
    // in this codebase (Map::clear() deliberately does not delete them), so queued pointers
    // cannot dangle.
    void DeferKeyFrameRelease(KeyFrame* const keyframe);

    // Same as DeferKeyFrameRelease, for a MapPoint whose SetBadFlag() just completed; its
    // descriptor is released at the next-plus-one Tick(). MapPoints registered in a Map are
    // never deleted (the only `delete pMP` in the codebase is for stereo-only temporal points,
    // which are never SetBadFlag()'d), so queued pointers cannot dangle.
    void DeferMapPointRelease(MapPoint* const map_point);

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

private:
    MemoryGovernor() = default;

    Atlas* mpAtlas = nullptr;
    std::atomic<long> mKfShellReleased{0};

    // Two-phase deferred-release queues (see the ORB_MEM_RECLAIM_BAD header note): objects
    // enqueued at tick T sit in mPending*, move to mReady* at Tick(T), and are released at
    // Tick(T+1) — strictly after the one frame that may still read them stock-legitimately.
    // reserve()d once on first enqueue; clear() (capacity-retaining) + swap thereafter, so the
    // steady-state per-tick path allocates nothing.
    std::vector<KeyFrame*> mPendingKeyFrameRelease;
    std::vector<KeyFrame*> mReadyKeyFrameRelease;
    std::vector<MapPoint*> mPendingMapPointRelease;
    std::vector<MapPoint*> mReadyMapPointRelease;
};

}  // namespace ORB_SLAM3

#endif  // MEMORYGOVERNOR_H
