/**
 * MemoryGovernor — env-gated periodic sampler of process memory and Atlas keyframe/map-point
 * bookkeeping, for the memory-reduction project's instrumentation phase (Task P0).
 *
 * ORB_MEM_STATS_CSV=<path>   enable sampling; one CSV row is appended to <path> every
 *                            ORB_MEM_STATS_EVERY processed frames. Unset => AppendStats() costs
 *                            a single static-bool check (zero per-frame overhead).
 * ORB_MEM_STATS_EVERY=<n>    sampling period in processed frames (default 30).
 *
 * P0 scope is measurement only: no eviction, no spill (Delta RSS = 0). The `kf_hot`, `kf_cold`,
 * `kf_shell`, `spill_mb` and `faultins_total` CSV columns are reserved for later phases and
 * always read 0 here — the header is written once, complete and stable, so downstream CSV
 * consumers do not break when those phases populate the columns.
 *
 * IMPORTANT for maintainers extending AppendStats(): this codebase's deterministic-mode
 * floating-point output is measurably sensitive to the process's heap allocation *pattern*
 * (bisected empirically — see MemoryGovernor.cc's ReadProcStatusFieldKb/WriteAll comments and the
 * P0 report). A *persistent* allocation added anywhere on the sampled-frame hot path (e.g. a
 * std::ofstream/std::string/std::vector that outlives the call, unlike the transient vectors
 * Atlas::GetAllMaps()/GetAllKeyFrames()/GetAllMapPoints() already return) is enough to change the
 * trajectory bit-for-bit versus stats disabled, even though nothing about the SLAM inputs
 * changed. The implementation therefore uses only raw POSIX fd I/O and fixed-size stack buffers
 * on that path — keep it that way (or re-verify the fast-gate md5 gate if you don't).
 */
#ifndef MEMORYGOVERNOR_H
#define MEMORYGOVERNOR_H

namespace ORB_SLAM3
{

class Atlas;

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

    // Current resident set size in KB, read from /proc/self/status VmRSS. 0 if unavailable.
    static long ReadVmRssKb();

    // Peak resident set size in KB, read from /proc/self/status VmHWM. 0 if unavailable.
    static long ReadVmHwmKb();

private:
    MemoryGovernor() = default;

    Atlas* mpAtlas = nullptr;
};

}  // namespace ORB_SLAM3

#endif  // MEMORYGOVERNOR_H
