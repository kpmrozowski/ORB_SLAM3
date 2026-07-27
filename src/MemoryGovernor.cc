#include "MemoryGovernor.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <unistd.h>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

#include <Atlas.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapPoint.h>
#include <PayloadSpill.h>

namespace ORB_SLAM3
{

namespace
{

// Env-var helpers, mirroring the ICEnvFlag/ICEnvDouble/ICEnvInt idiom in Tracking.cc (file-local,
// read-once-per-process statics; kept self-contained here rather than reused across files).
bool MemEnvSet(const char* const name)
{
    return std::getenv(name) != nullptr;
}

const char* MemEnvString(const char* const name)
{
    return std::getenv(name);
}

int MemEnvInt(const char* const name, const int fallback)
{
    const char* const value = std::getenv(name);
    return value != nullptr ? std::atoi(value) : fallback;
}

// Prints the ORB_MEM_RECLAIM_BAD/ORB_DETERMINISTIC misuse warning exactly once (called only from
// the function-local static initializer in ReclaimBadPayloadEnabled(), which C++11 guarantees
// runs at most once even under concurrent first-touch from multiple threads).
bool WarnIfReclaimRequestedWithoutDeterministic(const bool reclaim_requested,
                                                 const bool deterministic_mode)
{
    if (reclaim_requested && !deterministic_mode)
    {
        std::fprintf(stderr,
                      "ORB_MEM_RECLAIM_BAD=1 requires ORB_DETERMINISTIC=1 (deterministic mode); "
                      "ignoring -- reclaim stays inert.\n");
    }
    return reclaim_requested && deterministic_mode;
}

// Task P3d: same one-shot warning for the spill budget knob.
bool WarnIfBudgetRequestedWithoutDeterministic(const bool budget_requested,
                                               const bool deterministic_mode)
{
    if (budget_requested && !deterministic_mode)
    {
        std::fprintf(stderr,
                      "ORB_MEM_BUDGET_MB>0 requires ORB_DETERMINISTIC=1 (deterministic mode); "
                      "ignoring -- payload spill stays inert.\n");
    }
    return budget_requested && deterministic_mode;
}

// Task P6: same one-shot warning for the quarantine-delete knob.
bool WarnIfQuarantineRequestedWithoutDeterministic(const bool quarantine_requested,
                                                   const bool deterministic_mode)
{
    if (quarantine_requested && !deterministic_mode)
    {
        std::fprintf(stderr,
                      "ORB_MEM_DELETE_QUARANTINE>0 requires ORB_DETERMINISTIC=1 (deterministic "
                      "mode); ignoring -- culled MapPoints stay leaked.\n");
    }
    return quarantine_requested && deterministic_mode;
}

// Resolve the spill file path: ORB_MEM_SPILL, else <cwd>/orbmem_spill.bin.
std::string ResolveSpillPath()
{
    const char* const configured = std::getenv("ORB_MEM_SPILL");
    if (configured != nullptr && configured[0] != '\0')
    {
        return std::string(configured);
    }
    char cwd_buffer[4096];
    if (::getcwd(cwd_buffer, sizeof(cwd_buffer)) != nullptr)
    {
        return std::string(cwd_buffer) + "/orbmem_spill.bin";
    }
    return std::string("orbmem_spill.bin");
}

// Eviction ordering: coldest (lowest LRU tick) first, ties broken by ascending KeyFrame id so the
// oldest map region is paged out before newer covisible neighbours. No captures (global style).
bool EvictOrderLess(KeyFrame* const first, KeyFrame* const second)
{
    if (first->mSpillLastUseTick != second->mSpillLastUseTick)
    {
        return first->mSpillLastUseTick < second->mSpillLastUseTick;
    }
    return first->mnId < second->mnId;
}

// Writes the full buffer with a raw fd, looping over short writes (POSIX write() may write
// fewer bytes than requested). Best-effort: on error, drops the remaining bytes rather than
// looping forever -- this is a diagnostics sink, not something worth blocking tracking over.
void WriteAll(const int file_descriptor, const char* const data, const std::size_t length)
{
    std::size_t written = 0;
    while (written < length)
    {
        const ssize_t result = ::write(file_descriptor, data + written, length - written);
        if (result <= 0)
        {
            return;
        }
        written += static_cast<std::size_t>(result);
    }
}

// Reads one "Field:    1234 kB" field from /proc/self/status via raw POSIX I/O into a
// fixed-size stack buffer. Deliberately allocation-free (no std::ifstream/std::string): this
// codebase's deterministic-mode floating-point output is sensitive to the process's heap
// allocation pattern (see MemoryGovernor.h), so the sampler itself must not perturb it with the
// standard iostream/stringstream path's own internal (persistent, first-use) buffer allocation.
// Returns 0 if the field (or /proc itself) is unavailable, so callers degrade gracefully.
long ReadProcStatusFieldKb(const char* const field_name)
{
    const int status_fd = ::open("/proc/self/status", O_RDONLY);
    if (status_fd < 0)
    {
        return 0;
    }
    char buffer[8192];
    const ssize_t bytes_read = ::read(status_fd, buffer, sizeof(buffer) - 1);
    ::close(status_fd);
    if (bytes_read <= 0)
    {
        return 0;
    }
    buffer[bytes_read] = '\0';

    const char* const field_position = std::strstr(buffer, field_name);
    if (field_position == nullptr)
    {
        return 0;
    }
    return std::strtol(field_position + std::strlen(field_name), nullptr, 10);
}

}  // namespace

MemoryGovernor& MemoryGovernor::Instance()
{
    static MemoryGovernor instance;
    return instance;
}

void MemoryGovernor::SetAtlas(Atlas* const atlas)
{
    mpAtlas = atlas;
}

long MemoryGovernor::ReadVmRssKb()
{
    return ReadProcStatusFieldKb("VmRSS:");
}

long MemoryGovernor::ReadVmHwmKb()
{
    return ReadProcStatusFieldKb("VmHWM:");
}

void MemoryGovernor::AppendStats(const double timestamp_seconds)
{
    // Zero per-frame cost when disabled: one static-bool check, nothing else touched.
    static const bool stats_enabled = MemEnvSet("ORB_MEM_STATS_CSV");
    if (!stats_enabled)
    {
        return;
    }

    static const int sample_every_frames = MemEnvInt("ORB_MEM_STATS_EVERY", 30);
    static long frame_counter = 0;
    ++frame_counter;
    if (sample_every_frames <= 0 || frame_counter % sample_every_frames != 0)
    {
        return;
    }

    // Lazy raw fd, header written once, every row written (and thus durable, no separate flush
    // needed) immediately via ::write() -- mirrors append_frame_stats() in Tracking.cc (lazy
    // open, header once, survives a wall-clock-guard kill mid-run) but allocation-free; see the
    // ReadProcStatusFieldKb comment above for why that matters here.
    static int stats_fd = -1;
    if (stats_fd < 0)
    {
        stats_fd = ::open(MemEnvString("ORB_MEM_STATS_CSV"), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (stats_fd < 0)
        {
            return;
        }
        static const char* const header =
            "timestamp,vmrss_mb,vmhwm_mb,mallinfo_inuse_mb,mallinfo_free_mb,"
            "kf_created,kf_live,kf_hot,kf_cold,kf_shell,mp_created,mp_live,"
            "maps_stored,spill_mb,faultins_total,io_read_mb,io_write_mb,io_throttle_ms\n";
        WriteAll(stats_fd, header, std::strlen(header));
    }

    long mallinfo_inuse_mb = 0;
    long mallinfo_free_mb = 0;
#if defined(__GLIBC__)
    const struct mallinfo2 heap_info = mallinfo2();
    mallinfo_inuse_mb = static_cast<long>(heap_info.uordblks / (1024 * 1024));
    mallinfo_free_mb = static_cast<long>(heap_info.fordblks / (1024 * 1024));
#endif

    // Atlas-derived counters: only read through mpAtlas when SetAtlas() has actually been called
    // (deterministic single-threaded mode is the only mode the stats CSV runs in per the plan).
    // GetAllKeyFrames()/GetAllMapPoints() each return a freshly-allocated vector, but the
    // allocation is transient (freed before this function returns) rather than persistent, which
    // bisection confirmed does not perturb the trajectory the way the persistent stream buffer
    // above did.
    long kf_live = 0;
    long mp_live = 0;
    long maps_stored = 0;
    long kf_hot = 0;   // Task P3d: KeyFrames whose spillable payload is resident in RAM
    long kf_cold = 0;  // Task P3d: KeyFrames whose payload has been evicted to the spill file
    const bool spill_active = SpillActive();
    if (mpAtlas != nullptr)
    {
        for (Map* const current_map : mpAtlas->GetAllMaps())
        {
            if (current_map == nullptr)
            {
                continue;
            }
            const std::vector<KeyFrame*> map_key_frames = current_map->GetAllKeyFrames();
            kf_live += static_cast<long>(map_key_frames.size());
            mp_live += static_cast<long>(current_map->GetAllMapPoints().size());
            if (!current_map->IsInUse())
            {
                ++maps_stored;
            }
            if (spill_active)
            {
                for (KeyFrame* const key_frame : map_key_frames)
                {
                    if (key_frame == nullptr)
                    {
                        continue;
                    }
                    const std::uint8_t state =
                        key_frame->mSpillResidency.load(std::memory_order_acquire);
                    if (spill::PayloadPresent(state) || state == spill::kLoading)
                    {
                        ++kf_hot;
                    }
                    else
                    {
                        ++kf_cold;
                    }
                }
            }
        }
    }

    // kf_shell is live as of Task P1; spill_mb/faultins/io_* are live as of Task P3d (0 when the
    // spill worker was never started, i.e. the budget is unset).
    const long kf_shell = mKfShellReleased.load(std::memory_order_relaxed);
    long spill_mb = 0;
    long faultins_total = 0;
    double io_read_mb = 0.0;
    double io_write_mb = 0.0;
    double io_throttle_ms = 0.0;
    if (mSpillWorker != nullptr)
    {
        spill_mb = static_cast<long>(mSpillWorker->SpillBytes() / (1024 * 1024));
        faultins_total = mSpillWorker->SyncFaultins();
        io_read_mb = mSpillWorker->IoReadMb();
        io_write_mb = mSpillWorker->IoWriteMb();
        io_throttle_ms = mSpillWorker->IoThrottleMs();
    }

    char row[640];
    const int row_length = std::snprintf(
        row, sizeof(row),
        "%.6f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.3f,%.3f,%.3f\n",
        timestamp_seconds, ReadVmRssKb() / 1024, ReadVmHwmKb() / 1024, mallinfo_inuse_mb,
        mallinfo_free_mb, static_cast<long>(KeyFrame::nNextId), kf_live, kf_hot, kf_cold,
        kf_shell, static_cast<long>(MapPoint::nNextId), mp_live, maps_stored, spill_mb,
        faultins_total, io_read_mb, io_write_mb, io_throttle_ms);
    if (row_length > 0 && static_cast<std::size_t>(row_length) < sizeof(row))
    {
        WriteAll(stats_fd, row, static_cast<std::size_t>(row_length));
    }
}

void MemoryGovernor::SetTrackingWorkingSet(const std::vector<KeyFrame*>& local_key_frames,
                                           KeyFrame* const reference_key_frame)
{
    // Store non-owning observers only (see MemoryGovernor.h); consumed by the EvictionSweep() that
    // runs from the Tick() immediately following on the same thread. No allocation, no side effects.
    mWorkingSetLocalKeyFrames = &local_key_frames;
    mWorkingSetReferenceKeyFrame = reference_key_frame;
}

void MemoryGovernor::Tick()
{
    // Task P1 reclaim path (independent of the P3d spill path below): zero per-tick cost when
    // disabled -- one static-bool check via ReclaimBadPayloadEnabled()'s cached statics.
    if (ReclaimBadPayloadEnabled())
    {
        // Two-phase deferred release (see MemoryGovernor.h): free everything enqueued BEFORE the
        // previous tick (mReadyMapPointRelease), then rotate this tick's enqueues
        // (mPendingMapPointRelease) into mReadyMapPointRelease for the next tick. clear() retains
        // vector capacity, so after the reserve() in DeferMapPointRelease() this path performs no
        // heap allocation in steady state. KeyFrame release is no longer queued here (Task
        // P1-fix): it runs inline from KeyFrame::SetBadFlag() instead — see MemoryGovernor.h.
        for (MapPoint* const map_point : mReadyMapPointRelease)
        {
            map_point->ReleaseBadDescriptor();
        }
        mReadyMapPointRelease.clear();
        mReadyMapPointRelease.swap(mPendingMapPointRelease);

        static const int trim_every_ticks = 100;
        static long tick_counter = 0;
        ++tick_counter;
        if (tick_counter % trim_every_ticks == 0)
        {
#if defined(__GLIBC__)
            ::malloc_trim(0);
#endif
        }
    }

    // Task P3d cold-KF eviction sweep. Runs AFTER the LocalMapping/LoopClosing spins (System.cc),
    // so we never evict what this round's loop/reloc/merge detection just touched. Zero per-tick
    // cost when the budget is unset (one static-bool check in SpillActive()).
    if (SpillActive())
    {
        EvictionSweep();
    }

    // Task P6 quarantine-delete pass: delete culled MapPoints whose K-keyframe quarantine has
    // expired. Also runs at this same post-spin quiescent tick point, so no reader can race a
    // free. Zero per-tick cost when the quarantine is unset (one static-bool check).
    if (DeleteQuarantineActive())
    {
        DrainExpiredDeletes();
    }
}

bool MemoryGovernor::SpillActive()
{
    static const bool deterministic_mode = MemEnvSet("ORB_DETERMINISTIC");
    static const bool budget_requested = MemEnvInt("ORB_MEM_BUDGET_MB", 0) > 0;
    static const bool active =
        WarnIfBudgetRequestedWithoutDeterministic(budget_requested, deterministic_mode);
    return active;
}

void MemoryGovernor::EnsureSpillWorker()
{
    if (mSpillWorker != nullptr || mSpillDisabled)
    {
        return;
    }
    static const int io_mbps = MemEnvInt("ORB_MEM_IO_MBPS", 10);
    mSpillWorker.reset(new SpillWorker(ResolveSpillPath(), static_cast<double>(io_mbps)));
    if (!mSpillWorker->Ok())
    {
        std::fprintf(stderr, "MemoryGovernor: could not open spill file; payload spill disabled.\n");
        mSpillDisabled = true;
        mSpillWorker.reset();
    }
}

void MemoryGovernor::EvictionSweep()
{
    if (mpAtlas == nullptr)
    {
        return;
    }
    EnsureSpillWorker();
    if (mSpillWorker == nullptr)
    {
        return;
    }

    const long current_tick = mSpillTick.fetch_add(1, std::memory_order_relaxed) + 1;

    Map* const current_map = mpAtlas->GetCurrentMap();
    if (current_map == nullptr)
    {
        return;
    }

    // Protect set (Task P3d-evict) = { last W=60 created KeyFrames } ∪ { current
    // Tracking::mvpLocalKeyFrames } ∪ { reference KeyFrame's direct covisibles }. The recent window
    // is the mnId check below; the live local window + covisibles come from mProtectScratch, built
    // here from the working set System hands in each frame via SetTrackingWorkingSet(). Protecting
    // the ACTUAL working set (not just the most-recently-created ids) keeps the tracking/local-BA
    // set resident, collapsing the per-frame fault-in thrash to rare event-driven faults --
    // revisits/loops can make an OLD (low-mnId) KeyFrame the current working set, which the recent
    // window alone would evict and re-fault every frame. Eviction correctness does NOT depend on the
    // protect set -- any evicted KeyFrame faults back to identical bytes on the next read -- it only
    // bounds I/O churn on the active working set, so this stays a bit-identical residency-policy
    // change.
    PopulateProtectScratch();
    static const int hot_target = MemEnvInt("ORB_MEM_HOT_KFS", 150);
    static const long protect_window = 60;
    const long newest_id = static_cast<long>(KeyFrame::nNextId);

    long hot_count = 0;
    mEvictCandidatesScratch.clear();
    for (KeyFrame* const key_frame : current_map->GetAllKeyFrames())
    {
        if (key_frame == nullptr || !key_frame->SpillEligible())
        {
            continue;
        }
        const std::uint8_t state = key_frame->mSpillResidency.load(std::memory_order_acquire);
        const bool payload_in_ram = spill::PayloadPresent(state) || state == spill::kLoading;
        if (!payload_in_ram)
        {
            continue;  // already evicted / being loaded -- not occupying a hot slot to reclaim
        }
        ++hot_count;
        if (static_cast<long>(key_frame->mnId) >= newest_id - protect_window)
        {
            continue;  // protected recent window
        }
        if (mProtectScratch.count(key_frame) != 0)
        {
            continue;  // current tracking local window / reference covisibles -- keep resident
        }
        mEvictCandidatesScratch.push_back(key_frame);
    }

    if (hot_count <= hot_target)
    {
        return;
    }
    const long need_to_evict = hot_count - hot_target;

    std::sort(mEvictCandidatesScratch.begin(), mEvictCandidatesScratch.end(), EvictOrderLess);

    long freed = 0;
    long enqueued = 0;
    for (KeyFrame* const key_frame : mEvictCandidatesScratch)
    {
        const std::uint8_t state = key_frame->mSpillResidency.load(std::memory_order_acquire);
        if (freed < need_to_evict && state == spill::kPersistedResident)
        {
            // Durable on disk: free the RAM copy now (the only free point, main thread at tick).
            std::uint8_t expected = spill::kPersistedResident;
            if (key_frame->mSpillResidency.compare_exchange_strong(expected, spill::kEvicted,
                                                                   std::memory_order_acq_rel))
            {
                key_frame->SpillFree();
                ++freed;
            }
        }
        else if (enqueued < need_to_evict && state == spill::kResident)
        {
            // Not yet spilled: write-behind now, evict at a subsequent tick once durable.
            mSpillWorker->EnqueueWrite(key_frame);
            ++enqueued;
        }
    }
}

void MemoryGovernor::PopulateProtectScratch()
{
    mProtectScratch.clear();
    if (mWorkingSetLocalKeyFrames != nullptr)
    {
        for (KeyFrame* const local_key_frame : *mWorkingSetLocalKeyFrames)
        {
            if (local_key_frame != nullptr)
            {
                mProtectScratch.insert(local_key_frame);
            }
        }
    }
    if (mWorkingSetReferenceKeyFrame != nullptr && !mWorkingSetReferenceKeyFrame->isBad())
    {
        mProtectScratch.insert(mWorkingSetReferenceKeyFrame);
        for (KeyFrame* const covisible_key_frame :
             mWorkingSetReferenceKeyFrame->GetVectorCovisibleKeyFrames())
        {
            if (covisible_key_frame != nullptr)
            {
                mProtectScratch.insert(covisible_key_frame);
            }
        }
    }
}

void MemoryGovernor::FaultIn(KeyFrame* const key_frame)
{
    EnsureSpillWorker();
    if (mSpillWorker == nullptr)
    {
        return;
    }
    mSpillWorker->WaitResident(key_frame);
    key_frame->mSpillLastUseTick = mSpillTick.load(std::memory_order_relaxed);
}

void MemoryGovernor::Prefetch(KeyFrame* const key_frame)
{
    if (!SpillActive() || key_frame == nullptr)
    {
        return;
    }
    EnsureSpillWorker();
    if (mSpillWorker == nullptr)
    {
        return;
    }
    mSpillWorker->EnqueuePrefetch(key_frame);
}

void MemoryGovernor::ShutdownSpill()
{
    if (mSpillWorker != nullptr)
    {
        mSpillWorker->Shutdown();
    }
}

void MemoryGovernor::DeferMapPointRelease(MapPoint* const map_point)
{
    if (mPendingMapPointRelease.capacity() == 0)
    {
        mPendingMapPointRelease.reserve(4096);
        mReadyMapPointRelease.reserve(4096);
    }
    mPendingMapPointRelease.push_back(map_point);
}

void MemoryGovernor::DeferMapPointDelete(MapPoint* const map_point)
{
    // Stamp with the number of keyframes created so far (KeyFrame::nNextId). The quarantine in
    // DrainExpiredDeletes() is measured against how far that count advances, i.e. in KEYFRAMES,
    // which is the unit that bounds every multi-tick reader (mlpRecentAddedMapPoints in <=3 KFs;
    // see the DeferMapPointDelete() coverage note in MemoryGovernor.h). Appended in baddening
    // order -> intrinsically FIFO-sorted by this stamp (KeyFrame::nNextId never decreases within
    // a reset epoch), so DrainExpiredDeletes() can pop purely from the front.
    mDeleteQueue.emplace_back(map_point, static_cast<long>(KeyFrame::nNextId));
}

void MemoryGovernor::DrainExpiredDeletes()
{
    const long keyframes_now = static_cast<long>(KeyFrame::nNextId);
    if (keyframes_now < mLastKeyFrameNextIdSeen)
    {
        // KeyFrame::nNextId went backwards => a full Tracking::Reset() cleared the Atlas and
        // restarted the id counter. Every queued MapPoint was orphaned by that reset (the reset
        // does not delete MapPoints -- stock leaks them), and post-reset tracking state may still
        // reference them, so drop the queue AND the pending-scan batch WITHOUT freeing rather than
        // risk a use-after-free. The leaked remnant is bounded by the last <K keyframes' worth of
        // culls -- stock leaks all of them anyway. Resetting the MapPoints' mbDeleteQueued guards
        // is unnecessary: those objects are detached and will never be baddened (re-enqueued) again.
        mDeleteQueue.clear();
        mPendingDeleteScan.clear();
        mLastKeyFrameNextIdSeen = keyframes_now;
        return;
    }
    mLastKeyFrameNextIdSeen = keyframes_now;

    // Move every quarantine-expired MapPoint out of the FIFO into the pending-scan batch. It is NOT
    // freed here: SeverAndDeleteBatch() must first null every live-keyframe slot still pointing at
    // it (the two-prong root cause -- see MemoryGovernor.h). The queue is FIFO-ordered by baddening
    // keyframe id, so this pops purely from the front.
    static const long quarantine_keyframes =
        static_cast<long>(MemEnvInt("ORB_MEM_DELETE_QUARANTINE", 0));
    while (!mDeleteQueue.empty()
           && keyframes_now - mDeleteQueue.front().second >= quarantine_keyframes)
    {
        mPendingDeleteScan.push_back(mDeleteQueue.front().first);
        mDeleteQueue.pop_front();
    }

    // Amortize the O(keyframes * features) severance scan: run it (and the batch free) only once
    // every scan_every_ticks ticks, or immediately if the batch grew past scan_batch_cap (which
    // bounds the extra transient memory of already-culled shells held a little longer). Neither
    // constant affects the deterministic md5 -- the scan only nulls stale slots and frees memory,
    // both trajectory-invariant post-P0.5 -- so they are free tuning parameters (env-overridable
    // for measurement, sensible defaults otherwise).
    static const long scan_every_ticks =
        std::max(1L, static_cast<long>(MemEnvInt("ORB_MEM_DELETE_SCAN_EVERY", 20)));
    static const std::size_t scan_batch_cap =
        static_cast<std::size_t>(std::max(1, MemEnvInt("ORB_MEM_DELETE_SCAN_CAP", 8192)));
    ++mDeleteScanTick;
    const bool period_elapsed = (mDeleteScanTick % scan_every_ticks == 0);
    const bool batch_full = mPendingDeleteScan.size() >= scan_batch_cap;
    if (!mPendingDeleteScan.empty() && (period_elapsed || batch_full))
    {
        SeverAndDeleteBatch();
    }
}

void MemoryGovernor::SeverAndDeleteBatch()
{
    // Build the O(1)-membership doomed set from the batch (reused scratch; capacity retained).
    mDoomedScratch.clear();
    for (MapPoint* const doomed_map_point : mPendingDeleteScan)
    {
        mDoomedScratch.insert(doomed_map_point);
    }

    // Prong 1: null every live-keyframe mvpMapPoints slot still pointing at a batched MapPoint,
    // across every map in the atlas, BEFORE any free. GetAllKeyFrames() returns only live KFs
    // (SetBadFlag unlinks a bad KF from Map::mspKeyFrames), so this reaches exactly the live-and-
    // readable slots; shell-KF slots are kept out of the read path by Tracking's isBad() guards.
    if (mpAtlas != nullptr)
    {
        for (Map* const current_map : mpAtlas->GetAllMaps())
        {
            if (current_map == nullptr)
            {
                continue;
            }
            for (KeyFrame* const key_frame : current_map->GetAllKeyFrames())
            {
                if (key_frame != nullptr)
                {
                    key_frame->NullMapPointSlotsIn(mDoomedScratch);
                }
            }
        }
    }

    // The sanctioned owning free: no live/readable container references any of these MapPoints now
    // (Map::mspMapPoints erased at baddening, every live-KF slot nulled above, shell slots
    // unreadable, Frame/mpReplaced/mlpRecentAddedMapPoints readers cycled out over the quarantine).
    // mbDeleteQueued guaranteed each was enqueued -- hence delete()d -- exactly once.
    for (MapPoint* const doomed_map_point : mPendingDeleteScan)
    {
        delete doomed_map_point;
    }
    mPendingDeleteScan.clear();
}

bool MemoryGovernor::ReclaimBadPayloadEnabled()
{
    static const bool deterministic_mode = MemEnvSet("ORB_DETERMINISTIC");
    static const bool reclaim_requested = MemEnvInt("ORB_MEM_RECLAIM_BAD", 0) != 0;
    static const bool reclaim_enabled =
        WarnIfReclaimRequestedWithoutDeterministic(reclaim_requested, deterministic_mode);
    return reclaim_enabled;
}

bool MemoryGovernor::ParanoiaEnabled()
{
    static const bool enabled = MemEnvInt("ORB_MEM_PARANOIA", 0) != 0;
    return enabled;
}

bool MemoryGovernor::DeleteQuarantineActive()
{
    static const bool deterministic_mode = MemEnvSet("ORB_DETERMINISTIC");
    static const bool quarantine_requested = MemEnvInt("ORB_MEM_DELETE_QUARANTINE", 0) > 0;
    static const bool active =
        WarnIfQuarantineRequestedWithoutDeterministic(quarantine_requested, deterministic_mode);
    return active;
}

void MemoryGovernor::ConfigureAllocatorIfEnabled()
{
    if (!ReclaimBadPayloadEnabled())
    {
        return;
    }
#if defined(__GLIBC__)
    ::mallopt(M_ARENA_MAX, 2);
#endif
}

void MemoryGovernor::IncrementKfShellReleased()
{
    Instance().mKfShellReleased.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace ORB_SLAM3
