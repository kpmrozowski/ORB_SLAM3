/**
 * This file is part of ORB-SLAM3 (memory-reduction fork, env-gated, default OFF).
 *
 * Task P3d: cold-KeyFrame payload spill to an append-only binary file, served by a single
 * background I/O thread (SpillWorker). This header declares three cooperating pieces:
 *
 *   1. spill::* free functions  -- the fixed-layout, little-endian, memcpy-able record body
 *      (de)serialization and a self-contained crc32. These have NO dependency on KeyFrame or on
 *      the file, so they are unit-tested in isolation (Examples/Tests/payload_spill_roundtrip).
 *   2. PayloadSpill             -- owns the one write-once file + an in-RAM offset index
 *      (kf_id -> byte offset), a single mutex (touched at KF-Hz only) and per-record crc32 framing.
 *   3. SpillWorker              -- the single background thread doing ALL disk I/O: write-behind
 *      page-out (EnqueueWrite), read-ahead page-in (EnqueuePrefetch) and the synchronous cold-miss
 *      backstop (WaitResident, runs on the CALLER, never blocks the worker). A token bucket caps
 *      total read+write throughput at ORB_MEM_IO_MBPS (default 10 MB/s).
 *
 * NOT txt/csv/protobuf: the record body is raw fixed-shape numeric arrays (memcpy-in/memcpy-out)
 * with a crc32; a crc mismatch on read is a hard abort (never silent wrong data). See the plan's
 * decision 2b for why protobuf was rejected.
 *
 * Thread-safety model (deterministic mode only -- the feature is inert otherwise): in
 * ORB_DETERMINISTIC mode every SLAM operation (Track / LocalMapping / LoopClosing / culling /
 * SetBadFlag / the governor tick / the KeyFrame accessors) runs on ONE main thread; the SpillWorker
 * is the ONLY other thread. So the entire concurrency surface is main-thread-vs-worker, coordinated
 * by (a) a per-KeyFrame std::atomic<uint8_t> residency state with release/acquire ordering,
 * (b) the KeyFrame's existing mMutexFeatures around the actual payload field read/write, and
 * (c) payload frees happening ONLY on the main thread at the deterministic governor tick, and only
 * for records already durable on disk (kPersistedResident) -- so a free never races the worker.
 */

#ifndef ORB_SLAM3_PAYLOADSPILL_H
#define ORB_SLAM3_PAYLOADSPILL_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

namespace ORB_SLAM3
{

class KeyFrame;

namespace spill
{

// Per-KeyFrame residency state, stored in KeyFrame::mSpillResidency (std::atomic<uint8_t>). The
// three low values mean "payload bytes are present in RAM" (hot); the high values mean "not in RAM".
// State machine (all transitions release/acquire ordered):
//   kResident --EnqueueWrite--> kWriteQueued --(worker wrote)--> kPersistedResident
//   kPersistedResident --(governor tick free)--> kEvicted
//   kEvicted --EnqueuePrefetch--> kLoadQueued --(claimed)--> kLoading --(installed)--> kPersistedResident
//   kEvicted/kLoadQueued --WaitResident--> kLoading --(installed on caller)--> kPersistedResident
enum ResidencyState : std::uint8_t
{
    kResident = 0,           // in RAM, no disk copy yet
    kWriteQueued = 1,        // in RAM, write-behind enqueued (worker will serialize+write)
    kPersistedResident = 2,  // in RAM AND durable on disk -- the only eviction-eligible state
    kEvicted = 3,            // not in RAM, durable on disk -- must fault-in before read
    kLoadQueued = 4,         // not in RAM, prefetch enqueued
    kLoading = 5             // load in progress (claimed by worker or by a WaitResident caller)
};

inline bool PayloadPresent(const std::uint8_t state)
{
    return state == kResident || state == kWriteQueued || state == kPersistedResident;
}

// Self-contained CRC-32 (IEEE 802.3, poly 0xEDB88320), table built once on first call.
std::uint32_t Crc32(const std::uint8_t* const data, const std::size_t length);

// Record flags (record header `flags` field).
enum RecordFlag : std::uint32_t
{
    kFlagHasDesc = 1u << 0,
    kFlagHasKeysUn = 1u << 1,
    kFlagHasFeatVec = 1u << 2,
    kFlagIsMapPoints = 1u << 3,  // reserved for Task P4 stored-map MapPoint records
    kFlagLz4 = 1u << 4           // reserved for optional ORB_MEM_COMPRESS=lz4 (worker-side)
};

// Serialize a KeyFrame's spillable payload into a raw little-endian body (no header, no crc):
//   desc     : feature_count * 32 raw ORB descriptor bytes (CV_8U, 32 cols)
//   keysUn   : feature_count * 28 bytes (f32 x,y,size,angle,response; i32 octave,class_id)
//   featvec  : repeated (node_id u32, count u32, idx[count] u32) in ascending node_id order
// The three sections are concatenated in that order. Byte layout is fixed and host-endian
// (little-endian on the x86 dev box and ARM64 Pi target); records are single-run scratch, never
// shared across machines, so no endian conversion is performed.
void SerializePayloadBody(const cv::Mat& descriptors,
                          const std::vector<cv::KeyPoint>& keys_undistorted,
                          const DBoW2::FeatureVector& feature_vector,
                          std::vector<std::uint8_t>& out_body);

// Inverse of SerializePayloadBody. feature_count comes from the record header (== KeyFrame::N).
// Returns false on a malformed body (truncated section) so the caller can hard-abort with the id.
bool DeserializePayloadBody(const std::uint8_t* const body, const std::size_t body_length,
                            const int feature_count, cv::Mat& out_descriptors,
                            std::vector<cv::KeyPoint>& out_keys_undistorted,
                            DBoW2::FeatureVector& out_feature_vector);

}  // namespace spill

// RAII wrapper for a POSIX file descriptor (no raw owning pointer / manual close paths).
class ScopedFd
{
public:
    ScopedFd() = default;
    explicit ScopedFd(const int fd) : mFd(fd) {}
    ~ScopedFd();

    ScopedFd(const ScopedFd&) = delete;
    ScopedFd& operator=(const ScopedFd&) = delete;

    int Get() const { return mFd; }
    bool Valid() const { return mFd >= 0; }
    void Reset(const int fd);

private:
    int mFd = -1;
};

// The append-only, write-once binary spill file plus its in-RAM offset index. One instance per run,
// owned by the SpillWorker. All public methods are internally synchronized by mMutex (touched at
// KF-Hz only). File layout:
//   header: magic "OMSP0001" (8B) | version u32 | n_features u32 | flags u32 | pad u32   (24 bytes)
//   record: kf_id u64 | flags u32 | n u32 | raw_len u32 | comp_len u32 | crc32 u32 | pad u32 (32B)
//           followed by comp_len payload bytes (== raw_len; no compression by default)
class PayloadSpill
{
public:
    static constexpr std::size_t kFileHeaderBytes = 24;
    static constexpr std::size_t kRecordHeaderBytes = 32;

    explicit PayloadSpill(const std::string& path);
    ~PayloadSpill() = default;

    PayloadSpill(const PayloadSpill&) = delete;
    PayloadSpill& operator=(const PayloadSpill&) = delete;

    bool Ok() const { return mFd.Valid(); }

    // Append a record for kf_id (write-once; a second write for the same id keeps the first record
    // and returns true without rewriting). Returns false on I/O error.
    bool WriteRecord(const std::uint64_t kf_id, const int feature_count,
                     const std::vector<std::uint8_t>& body);

    // Read the record for kf_id into out_body (payload bytes only). Verifies crc32; on mismatch or
    // a missing record, prints the kf_id and aborts (never silent wrong data). Returns false only
    // if the record is genuinely absent (caller decides -- currently a hard error upstream).
    bool ReadRecord(const std::uint64_t kf_id, const int feature_count,
                    std::vector<std::uint8_t>& out_body);

    bool HasRecord(const std::uint64_t kf_id) const;
    std::uint64_t BytesWritten() const;

private:
    mutable std::mutex mMutex;
    ScopedFd mFd;
    std::uint64_t mWriteCursor = kFileHeaderBytes;  // next append offset
    std::unordered_map<std::uint64_t, std::uint64_t> mOffsetIndex;  // kf_id -> record header offset
};

// The single background I/O thread. Owned by MemoryGovernor, started lazily when ORB_MEM_BUDGET_MB
// enables the feature. See the header comment for the thread-safety model.
class SpillWorker
{
public:
    SpillWorker(const std::string& spill_path, const double io_mbps);
    ~SpillWorker();

    SpillWorker(const SpillWorker&) = delete;
    SpillWorker& operator=(const SpillWorker&) = delete;

    bool Ok() const { return mSpill.Ok(); }

    // Write-behind page-out: enqueue a resident, never-yet-spilled KeyFrame for background
    // serialize+write. No-op unless the KF is in kResident. Never blocks.
    void EnqueueWrite(KeyFrame* const key_frame);

    // Read-ahead page-in: enqueue an evicted KeyFrame for background load. No-op unless the KF is
    // in kEvicted. Never blocks.
    void EnqueuePrefetch(KeyFrame* const key_frame);

    // Cold-miss backstop: make the KeyFrame's payload resident, doing the read on THIS (caller)
    // thread if needed -- never blocks the worker. Returns with residency == kPersistedResident.
    void WaitResident(KeyFrame* const key_frame);

    // Stop the worker thread and join it. Idempotent. Called from System::Shutdown().
    void Shutdown();

    // Stats (all monotonic; safe to read from the main thread at any time).
    double IoReadMb() const;
    double IoWriteMb() const;
    double IoThrottleMs() const;
    long SyncFaultins() const { return mSyncFaultins.load(std::memory_order_relaxed); }
    std::uint64_t SpillBytes() const { return mSpill.BytesWritten(); }

private:
    void WorkerLoop();
    void DoWrite(KeyFrame* const key_frame);
    // Load `key_frame`'s payload from disk and install it. `throttle` drains the token bucket
    // (true for both worker and caller paths -- all I/O is capped). Sets residency to
    // kPersistedResident and notifies load waiters.
    void LoadAndInstall(KeyFrame* const key_frame);
    // Blocking token acquire for `bytes`; accumulates throttled time into mThrottleMs.
    void AcquireTokens(const std::size_t bytes);

    PayloadSpill mSpill;

    // Token bucket (bytes). Guarded by mBucketMutex.
    std::mutex mBucketMutex;
    double mTokens;
    double mCapacityBytes;
    double mRateBytesPerSec;
    std::chrono::steady_clock::time_point mLastRefill;

    // Work queues + wakeup. Guarded by mQueueMutex.
    std::mutex mQueueMutex;
    std::condition_variable mQueueCv;
    std::deque<KeyFrame*> mWriteQueue;
    std::deque<KeyFrame*> mPrefetchQueue;
    bool mStop = false;

    // Load-completion signalling (for a WaitResident caller that finds another agent mid-load).
    std::mutex mLoadMutex;
    std::condition_variable mLoadCv;

    std::thread mThread;

    // Stats.
    std::atomic<std::uint64_t> mReadBytes{0};
    std::atomic<std::uint64_t> mWriteBytes{0};
    std::atomic<long long> mThrottleMicros{0};
    std::atomic<long> mSyncFaultins{0};
};

}  // namespace ORB_SLAM3

#endif  // ORB_SLAM3_PAYLOADSPILL_H
