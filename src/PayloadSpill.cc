/**
 * This file is part of ORB-SLAM3 (memory-reduction fork, env-gated, default OFF). See
 * include/PayloadSpill.h for the design and the thread-safety model.
 */

#include "PayloadSpill.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>

#include <KeyFrame.h>

namespace ORB_SLAM3
{

namespace spill
{

namespace
{

std::array<std::uint32_t, 256> MakeCrcTable()
{
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t index = 0; index < 256; ++index)
    {
        std::uint32_t value = index;
        for (int bit = 0; bit < 8; ++bit)
        {
            value = (value & 1u) ? (0xEDB88320u ^ (value >> 1)) : (value >> 1);
        }
        table[index] = value;
    }
    return table;
}

// Append `count` little-endian bytes of `value` (host is little-endian on all supported targets).
template <typename ValueType>
void AppendRaw(std::vector<std::uint8_t>& out, const ValueType value)
{
    const std::uint8_t* const bytes = reinterpret_cast<const std::uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(ValueType));
}

}  // namespace

std::uint32_t Crc32(const std::uint8_t* const data, const std::size_t length)
{
    static const std::array<std::uint32_t, 256> table = MakeCrcTable();
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t offset = 0; offset < length; ++offset)
    {
        crc = table[(crc ^ data[offset]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

void SerializePayloadBody(const cv::Mat& descriptors,
                          const std::vector<cv::KeyPoint>& keys_undistorted,
                          const DBoW2::FeatureVector& feature_vector,
                          std::vector<std::uint8_t>& out_body)
{
    out_body.clear();

    // desc: n * 32 raw ORB rows. The KeyFrame's descriptor Mat is a clone (continuous), CV_8U,
    // 32 cols; append it row-major.
    if (!descriptors.empty())
    {
        const std::size_t desc_bytes = static_cast<std::size_t>(descriptors.rows) *
                                       static_cast<std::size_t>(descriptors.cols);
        const std::uint8_t* const desc_data = descriptors.ptr<std::uint8_t>(0);
        out_body.insert(out_body.end(), desc_data, desc_data + desc_bytes);
    }

    // keysUn: n * 28 bytes.
    for (const cv::KeyPoint& keypoint : keys_undistorted)
    {
        AppendRaw(out_body, keypoint.pt.x);
        AppendRaw(out_body, keypoint.pt.y);
        AppendRaw(out_body, keypoint.size);
        AppendRaw(out_body, keypoint.angle);
        AppendRaw(out_body, keypoint.response);
        AppendRaw(out_body, static_cast<std::int32_t>(keypoint.octave));
        AppendRaw(out_body, static_cast<std::int32_t>(keypoint.class_id));
    }

    // featvec: repeated (node_id u32, count u32, idx[count] u32) in ascending node_id order.
    for (DBoW2::FeatureVector::const_iterator node_it = feature_vector.begin();
         node_it != feature_vector.end(); ++node_it)
    {
        AppendRaw(out_body, static_cast<std::uint32_t>(node_it->first));
        AppendRaw(out_body, static_cast<std::uint32_t>(node_it->second.size()));
        for (const unsigned int feature_index : node_it->second)
        {
            AppendRaw(out_body, static_cast<std::uint32_t>(feature_index));
        }
    }
}

bool DeserializePayloadBody(const std::uint8_t* const body, const std::size_t body_length,
                            const int feature_count, cv::Mat& out_descriptors,
                            std::vector<cv::KeyPoint>& out_keys_undistorted,
                            DBoW2::FeatureVector& out_feature_vector)
{
    const std::size_t count = static_cast<std::size_t>(feature_count);
    const std::size_t desc_bytes = count * 32u;
    const std::size_t keysun_bytes = count * 28u;
    if (body_length < desc_bytes + keysun_bytes)
    {
        return false;
    }

    std::size_t cursor = 0;

    // desc
    out_descriptors.create(feature_count, 32, CV_8U);
    std::memcpy(out_descriptors.ptr<std::uint8_t>(0), body + cursor, desc_bytes);
    cursor += desc_bytes;

    // keysUn
    out_keys_undistorted.clear();
    out_keys_undistorted.resize(count);
    for (std::size_t index = 0; index < count; ++index)
    {
        cv::KeyPoint& keypoint = out_keys_undistorted[index];
        std::memcpy(&keypoint.pt.x, body + cursor + 0, 4);
        std::memcpy(&keypoint.pt.y, body + cursor + 4, 4);
        std::memcpy(&keypoint.size, body + cursor + 8, 4);
        std::memcpy(&keypoint.angle, body + cursor + 12, 4);
        std::memcpy(&keypoint.response, body + cursor + 16, 4);
        std::int32_t octave = 0;
        std::int32_t class_id = 0;
        std::memcpy(&octave, body + cursor + 20, 4);
        std::memcpy(&class_id, body + cursor + 24, 4);
        keypoint.octave = octave;
        keypoint.class_id = class_id;
        cursor += 28;
    }

    // featvec: parse until the remaining body is consumed.
    out_feature_vector.clear();
    while (cursor + 8 <= body_length)
    {
        std::uint32_t node_id = 0;
        std::uint32_t entry_count = 0;
        std::memcpy(&node_id, body + cursor, 4);
        std::memcpy(&entry_count, body + cursor + 4, 4);
        cursor += 8;
        const std::size_t entry_bytes = static_cast<std::size_t>(entry_count) * 4u;
        if (cursor + entry_bytes > body_length)
        {
            return false;
        }
        std::vector<unsigned int> feature_indices(entry_count);
        for (std::uint32_t entry = 0; entry < entry_count; ++entry)
        {
            std::uint32_t feature_index = 0;
            std::memcpy(&feature_index, body + cursor, 4);
            cursor += 4;
            feature_indices[entry] = feature_index;
        }
        out_feature_vector[node_id] = std::move(feature_indices);
    }

    return cursor == body_length;
}

}  // namespace spill

ScopedFd::~ScopedFd()
{
    if (mFd >= 0)
    {
        ::close(mFd);
    }
}

void ScopedFd::Reset(const int fd)
{
    if (mFd >= 0)
    {
        ::close(mFd);
    }
    mFd = fd;
}

namespace
{

// Loop over short writes/reads (POSIX pwrite/pread may transfer fewer bytes than requested).
bool PwriteAll(const int fd, const std::uint8_t* const data, const std::size_t length,
               const std::uint64_t offset)
{
    std::size_t done = 0;
    while (done < length)
    {
        const ssize_t result = ::pwrite(fd, data + done, length - done, static_cast<off_t>(offset + done));
        if (result <= 0)
        {
            return false;
        }
        done += static_cast<std::size_t>(result);
    }
    return true;
}

bool PreadAll(const int fd, std::uint8_t* const data, const std::size_t length,
              const std::uint64_t offset)
{
    std::size_t done = 0;
    while (done < length)
    {
        const ssize_t result = ::pread(fd, data + done, length - done, static_cast<off_t>(offset + done));
        if (result <= 0)
        {
            return false;
        }
        done += static_cast<std::size_t>(result);
    }
    return true;
}

}  // namespace

PayloadSpill::PayloadSpill(const std::string& path)
{
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    mFd.Reset(fd);
    if (!mFd.Valid())
    {
        std::fprintf(stderr, "PayloadSpill: failed to open spill file '%s'\n", path.c_str());
        return;
    }
    std::uint8_t header[kFileHeaderBytes] = {0};
    std::memcpy(header, "OMSP0001", 8);
    const std::uint32_t version = 1;
    std::memcpy(header + 8, &version, 4);  // n_features / flags / pad left zero (per-record instead)
    PwriteAll(mFd.Get(), header, kFileHeaderBytes, 0);
}

bool PayloadSpill::WriteRecord(const std::uint64_t kf_id, const int feature_count,
                               const std::vector<std::uint8_t>& body)
{
    std::lock_guard<std::mutex> lock(mMutex);
    if (!mFd.Valid())
    {
        return false;
    }
    if (mOffsetIndex.find(kf_id) != mOffsetIndex.end())
    {
        return true;  // write-once: payload is immutable after ComputeBoW
    }

    const std::uint32_t raw_len = static_cast<std::uint32_t>(body.size());
    const std::uint32_t crc = spill::Crc32(body.data(), body.size());

    std::uint8_t record_header[kRecordHeaderBytes] = {0};
    const std::uint32_t flags =
        spill::kFlagHasDesc | spill::kFlagHasKeysUn | spill::kFlagHasFeatVec;
    const std::uint32_t n = static_cast<std::uint32_t>(feature_count);
    std::memcpy(record_header + 0, &kf_id, 8);
    std::memcpy(record_header + 8, &flags, 4);
    std::memcpy(record_header + 12, &n, 4);
    std::memcpy(record_header + 16, &raw_len, 4);
    std::memcpy(record_header + 20, &raw_len, 4);  // comp_len == raw_len (no compression)
    std::memcpy(record_header + 24, &crc, 4);

    const std::uint64_t offset = mWriteCursor;
    if (!PwriteAll(mFd.Get(), record_header, kRecordHeaderBytes, offset))
    {
        return false;
    }
    if (raw_len > 0 && !PwriteAll(mFd.Get(), body.data(), body.size(), offset + kRecordHeaderBytes))
    {
        return false;
    }
    mOffsetIndex.emplace(kf_id, offset);
    mWriteCursor = offset + kRecordHeaderBytes + raw_len;
    return true;
}

bool PayloadSpill::ReadRecord(const std::uint64_t kf_id, const int feature_count,
                              std::vector<std::uint8_t>& out_body)
{
    std::uint64_t offset = 0;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        const std::unordered_map<std::uint64_t, std::uint64_t>::const_iterator found =
            mOffsetIndex.find(kf_id);
        if (found == mOffsetIndex.end())
        {
            return false;
        }
        offset = found->second;
    }

    std::uint8_t record_header[kRecordHeaderBytes] = {0};
    if (!PreadAll(mFd.Get(), record_header, kRecordHeaderBytes, offset))
    {
        std::fprintf(stderr, "PayloadSpill: short read of record header for kf_id=%llu\n",
                     static_cast<unsigned long long>(kf_id));
        std::abort();
    }
    std::uint32_t stored_n = 0;
    std::uint32_t raw_len = 0;
    std::uint32_t comp_len = 0;
    std::uint32_t stored_crc = 0;
    std::memcpy(&stored_n, record_header + 12, 4);
    std::memcpy(&raw_len, record_header + 16, 4);
    std::memcpy(&comp_len, record_header + 20, 4);
    std::memcpy(&stored_crc, record_header + 24, 4);
    if (stored_n != static_cast<std::uint32_t>(feature_count))
    {
        std::fprintf(stderr, "PayloadSpill: feature-count mismatch for kf_id=%llu (%u vs %d)\n",
                     static_cast<unsigned long long>(kf_id), stored_n, feature_count);
        std::abort();
    }

    out_body.resize(comp_len);
    if (comp_len > 0 && !PreadAll(mFd.Get(), out_body.data(), comp_len, offset + kRecordHeaderBytes))
    {
        std::fprintf(stderr, "PayloadSpill: short read of record body for kf_id=%llu\n",
                     static_cast<unsigned long long>(kf_id));
        std::abort();
    }

    const std::uint32_t actual_crc = spill::Crc32(out_body.data(), out_body.size());
    if (actual_crc != stored_crc)
    {
        std::fprintf(stderr, "PayloadSpill: CRC MISMATCH for kf_id=%llu (stored=%08x actual=%08x)\n",
                     static_cast<unsigned long long>(kf_id), stored_crc, actual_crc);
        std::abort();
    }
    return true;
}

bool PayloadSpill::HasRecord(const std::uint64_t kf_id) const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mOffsetIndex.find(kf_id) != mOffsetIndex.end();
}

std::uint64_t PayloadSpill::BytesWritten() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mWriteCursor - kFileHeaderBytes;
}

// ---------------------------------------------------------------------------------------------
// SpillWorker
// ---------------------------------------------------------------------------------------------

SpillWorker::SpillWorker(const std::string& spill_path, const double io_mbps)
    : mSpill(spill_path)
{
    const double rate = (io_mbps > 0.0 ? io_mbps : 10.0) * 1024.0 * 1024.0;
    mRateBytesPerSec = rate;
    mCapacityBytes = rate;  // 1 second of burst
    mTokens = rate;
    mLastRefill = std::chrono::steady_clock::now();
    if (mSpill.Ok())
    {
        mThread = std::thread(&SpillWorker::WorkerLoop, this);
    }
}

SpillWorker::~SpillWorker()
{
    Shutdown();
}

void SpillWorker::Shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mQueueMutex);
        if (mStop)
        {
            return;
        }
        mStop = true;
    }
    mQueueCv.notify_all();
    if (mThread.joinable())
    {
        mThread.join();
    }
}

void SpillWorker::AcquireTokens(const std::size_t bytes)
{
    const double requested = static_cast<double>(bytes);
    std::unique_lock<std::mutex> lock(mBucketMutex);
    while (true)
    {
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        const double elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - mLastRefill).count();
        mLastRefill = now;
        mTokens += elapsed_seconds * mRateBytesPerSec;
        if (mTokens > mCapacityBytes)
        {
            mTokens = mCapacityBytes;
        }
        // Grant threshold caps at the bucket capacity so a single record larger than the bucket
        // still drains (tokens go negative and are repaid over time, preserving the average rate).
        const double threshold = requested < mCapacityBytes ? requested : mCapacityBytes;
        if (mTokens >= threshold)
        {
            mTokens -= requested;
            return;
        }
        const double sleep_seconds = (threshold - mTokens) / mRateBytesPerSec;
        lock.unlock();
        const std::chrono::steady_clock::time_point sleep_start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds));
        const std::chrono::steady_clock::time_point sleep_end = std::chrono::steady_clock::now();
        mThrottleMicros.fetch_add(
            std::chrono::duration_cast<std::chrono::microseconds>(sleep_end - sleep_start).count(),
            std::memory_order_relaxed);
        lock.lock();
    }
}

void SpillWorker::EnqueueWrite(KeyFrame* const key_frame)
{
    std::uint8_t expected = spill::kResident;
    if (!key_frame->mSpillResidency.compare_exchange_strong(
            expected, spill::kWriteQueued, std::memory_order_acq_rel))
    {
        return;  // not in a fresh-resident state; nothing to schedule
    }
    {
        std::lock_guard<std::mutex> lock(mQueueMutex);
        mWriteQueue.push_back(key_frame);
    }
    mQueueCv.notify_one();
}

void SpillWorker::EnqueuePrefetch(KeyFrame* const key_frame)
{
    std::uint8_t expected = spill::kEvicted;
    if (!key_frame->mSpillResidency.compare_exchange_strong(
            expected, spill::kLoadQueued, std::memory_order_acq_rel))
    {
        return;  // already resident, or another agent owns the transition
    }
    {
        std::lock_guard<std::mutex> lock(mQueueMutex);
        mPrefetchQueue.push_back(key_frame);
    }
    mQueueCv.notify_one();
}

void SpillWorker::WaitResident(KeyFrame* const key_frame)
{
    while (true)
    {
        std::uint8_t state = key_frame->mSpillResidency.load(std::memory_order_acquire);
        if (spill::PayloadPresent(state))
        {
            return;
        }
        if (state == spill::kEvicted)
        {
            std::uint8_t expected = spill::kEvicted;
            if (key_frame->mSpillResidency.compare_exchange_strong(
                    expected, spill::kLoading, std::memory_order_acq_rel))
            {
                mSyncFaultins.fetch_add(1, std::memory_order_relaxed);
                LoadAndInstall(key_frame);
                return;
            }
            continue;  // lost the race; re-read state
        }
        if (state == spill::kLoadQueued)
        {
            std::uint8_t expected = spill::kLoadQueued;
            if (key_frame->mSpillResidency.compare_exchange_strong(
                    expected, spill::kLoading, std::memory_order_acq_rel))
            {
                // Steal the queued load onto the caller thread (rare); the worker will skip it when
                // it later pops the stale queue entry and finds the state no longer kLoadQueued.
                mSyncFaultins.fetch_add(1, std::memory_order_relaxed);
                LoadAndInstall(key_frame);
                return;
            }
            continue;
        }
        // state == kLoading: another agent (worker or a concurrent caller) is loading it. Wait.
        std::unique_lock<std::mutex> lock(mLoadMutex);
        mLoadCv.wait(lock, [key_frame] {
            return spill::PayloadPresent(
                key_frame->mSpillResidency.load(std::memory_order_acquire));
        });
        return;
    }
}

void SpillWorker::LoadAndInstall(KeyFrame* const key_frame)
{
    std::vector<std::uint8_t> body;
    const int feature_count = key_frame->SpillFeatureCount();
    const std::size_t expected_bytes =
        static_cast<std::size_t>(feature_count) * 60u + PayloadSpill::kRecordHeaderBytes;
    AcquireTokens(expected_bytes);
    if (!mSpill.ReadRecord(key_frame->mnId, feature_count, body))
    {
        std::fprintf(stderr, "SpillWorker: no spill record for kf_id=%llu on fault-in\n",
                     static_cast<unsigned long long>(key_frame->mnId));
        std::abort();
    }
    mReadBytes.fetch_add(body.size(), std::memory_order_relaxed);
    key_frame->SpillInstall(body);
    key_frame->mSpillResidency.store(spill::kPersistedResident, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(mLoadMutex);
    }
    mLoadCv.notify_all();
}

void SpillWorker::DoWrite(KeyFrame* const key_frame)
{
    std::uint8_t state = key_frame->mSpillResidency.load(std::memory_order_acquire);
    if (state != spill::kWriteQueued)
    {
        return;  // state changed under us (e.g. faulted elsewhere); nothing to write
    }
    std::vector<std::uint8_t> body;
    if (!key_frame->SpillSerialize(body))
    {
        // KeyFrame went bad / payload released before we serialized it: no valid record. It is
        // still resident for the fields SpillSerialize would have read (keysUn is kept by
        // ReleaseBadPayload), so return it to a resident state and never spill it.
        std::uint8_t expected = spill::kWriteQueued;
        key_frame->mSpillResidency.compare_exchange_strong(expected, spill::kResident,
                                                           std::memory_order_acq_rel);
        return;
    }
    AcquireTokens(body.size() + PayloadSpill::kRecordHeaderBytes);
    if (!mSpill.WriteRecord(key_frame->mnId, key_frame->SpillFeatureCount(), body))
    {
        std::fprintf(stderr, "SpillWorker: write failed for kf_id=%llu\n",
                     static_cast<unsigned long long>(key_frame->mnId));
        std::uint8_t expected = spill::kWriteQueued;
        key_frame->mSpillResidency.compare_exchange_strong(expected, spill::kResident,
                                                           std::memory_order_acq_rel);
        return;
    }
    mWriteBytes.fetch_add(body.size(), std::memory_order_relaxed);
    std::uint8_t expected = spill::kWriteQueued;
    key_frame->mSpillResidency.compare_exchange_strong(expected, spill::kPersistedResident,
                                                       std::memory_order_acq_rel);
}

void SpillWorker::WorkerLoop()
{
    while (true)
    {
        KeyFrame* write_target = nullptr;
        KeyFrame* prefetch_target = nullptr;
        {
            std::unique_lock<std::mutex> lock(mQueueMutex);
            mQueueCv.wait(lock, [this] {
                return mStop || !mWriteQueue.empty() || !mPrefetchQueue.empty();
            });
            // On shutdown, drop any still-queued work: the spill file is single-run scratch and the
            // process is exiting (trajectory already saved from resident poses), so unwritten
            // records and unfinished prefetches are irrelevant -- exit promptly instead.
            if (mStop)
            {
                return;
            }
            if (!mWriteQueue.empty())
            {
                write_target = mWriteQueue.front();
                mWriteQueue.pop_front();
            }
            else if (!mPrefetchQueue.empty())
            {
                prefetch_target = mPrefetchQueue.front();
                mPrefetchQueue.pop_front();
            }
        }
        if (write_target != nullptr)
        {
            DoWrite(write_target);
        }
        else if (prefetch_target != nullptr)
        {
            std::uint8_t expected = spill::kLoadQueued;
            if (prefetch_target->mSpillResidency.compare_exchange_strong(
                    expected, spill::kLoading, std::memory_order_acq_rel))
            {
                LoadAndInstall(prefetch_target);
            }
            // else: a WaitResident caller already claimed/loaded it -- skip the stale entry.
        }
    }
}

double SpillWorker::IoReadMb() const
{
    return static_cast<double>(mReadBytes.load(std::memory_order_relaxed)) / (1024.0 * 1024.0);
}

double SpillWorker::IoWriteMb() const
{
    return static_cast<double>(mWriteBytes.load(std::memory_order_relaxed)) / (1024.0 * 1024.0);
}

double SpillWorker::IoThrottleMs() const
{
    return static_cast<double>(mThrottleMicros.load(std::memory_order_relaxed)) / 1000.0;
}

}  // namespace ORB_SLAM3
