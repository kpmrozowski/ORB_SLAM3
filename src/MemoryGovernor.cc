#include "MemoryGovernor.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

#include <Atlas.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapPoint.h>

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
            "maps_stored,spill_mb,faultins_total\n";
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
    if (mpAtlas != nullptr)
    {
        for (Map* const current_map : mpAtlas->GetAllMaps())
        {
            if (current_map == nullptr)
            {
                continue;
            }
            kf_live += static_cast<long>(current_map->GetAllKeyFrames().size());
            mp_live += static_cast<long>(current_map->GetAllMapPoints().size());
            if (!current_map->IsInUse())
            {
                ++maps_stored;
            }
        }
    }

    // Reserved for later phases (P1+ eviction/spill); always 0 here. Kept as named constants
    // (rather than inline literals) so it is obvious at the call site which columns are stubs.
    const long kf_hot_stub = 0;
    const long kf_cold_stub = 0;
    const long kf_shell_stub = 0;
    const long spill_mb_stub = 0;
    const long faultins_total_stub = 0;

    char row[512];
    const int row_length = std::snprintf(
        row, sizeof(row), "%.6f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
        timestamp_seconds, ReadVmRssKb() / 1024, ReadVmHwmKb() / 1024, mallinfo_inuse_mb,
        mallinfo_free_mb, static_cast<long>(KeyFrame::nNextId), kf_live, kf_hot_stub, kf_cold_stub,
        kf_shell_stub, static_cast<long>(MapPoint::nNextId), mp_live, maps_stored, spill_mb_stub,
        faultins_total_stub);
    if (row_length > 0 && static_cast<std::size_t>(row_length) < sizeof(row))
    {
        WriteAll(stats_fd, row, static_cast<std::size_t>(row_length));
    }
}

}  // namespace ORB_SLAM3
