/**
 * This file is part of the offline-evaluation patches for ORB-SLAM3 (env-gated, default OFF).
 * See include/FeaturePrefetcher.h for the design.
 */

#include "FeaturePrefetcher.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ORBextractor.h"

namespace ORB_SLAM3
{

FeaturePrefetcher* FeaturePrefetcher::sRegistered = nullptr;

namespace
{
float ReadClaheClip()
{
    const char* const clipEnv = std::getenv("ORB_CLAHE");
    return clipEnv != nullptr ? static_cast<float>(std::atof(clipEnv)) : 0.0f;
}

int ReadClaheTile()
{
    const char* const tileEnv = std::getenv("ORB_CLAHE_TILE");
    return tileEnv != nullptr ? std::atoi(tileEnv) : 8;
}
}  // namespace

FeaturePrefetcher::FeaturePrefetcher(const std::vector<std::string>& imagePaths,
                                     const std::vector<double>& timestamps,
                                     const ORBextractor* normalExtractor, const int numWorkers,
                                     const bool rgb)
    : mImagePaths(imagePaths),
      mTimestamps(timestamps),
      mNormalExtractor(normalExtractor),
      mNumFrames(static_cast<int>(imagePaths.size())),
      mMaxAhead(std::max(4 * numWorkers, 32)),
      mRGB(rgb),
      mFeatures(normalExtractor->GetFeatures()),
      mScaleFactor(normalExtractor->GetScaleFactor()),
      mLevels(normalExtractor->GetLevels()),
      mIniThFAST(normalExtractor->GetIniThFAST()),
      mMinThFAST(normalExtractor->GetMinThFAST()),
      mClaheClip(ReadClaheClip()),
      mClaheTile(ReadClaheTile())
{
    const int workerCount = std::max(1, numWorkers);
    mWorkers.reserve(workerCount);
    for (int worker = 0; worker < workerCount; ++worker)
    {
        mWorkers.emplace_back(&FeaturePrefetcher::WorkerLoop, this);
    }
    std::cout << "FeaturePrefetcher: " << workerCount << " worker(s), " << mNumFrames
              << " frames, maxAhead " << mMaxAhead << (mClaheClip > 0.0f ? " (CLAHE)" : "")
              << std::endl;
}

FeaturePrefetcher::~FeaturePrefetcher()
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mStop = true;
    }
    mScheduleCv.notify_all();
    mReadyCv.notify_all();
    for (std::thread& worker : mWorkers)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
    if (sRegistered == this)
    {
        sRegistered = nullptr;
    }
}

cv::Mat FeaturePrefetcher::LoadAndPreprocess(const int index) const
{
    // Mirror Tracking::GrabImageMonocular exactly: imread UNCHANGED, then the same grayscale
    // branch and the same optional CLAHE, so the extractor sees a byte-identical image.
    cv::Mat image = cv::imread(mImagePaths[index], cv::IMREAD_UNCHANGED);
    if (image.empty())
    {
        return image;
    }
    if (image.channels() == 3)
    {
        cv::cvtColor(image, image, mRGB ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4)
    {
        cv::cvtColor(image, image, mRGB ? cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY);
    }
    if (mClaheClip > 0.0f)
    {
        // Each worker owns its own CLAHE instance (apply() is not safe to share across threads).
        const cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(mClaheClip, cv::Size(mClaheTile, mClaheTile));
        clahe->apply(image, image);
    }
    return image;
}

void FeaturePrefetcher::WorkerLoop()
{
    // One extractor per worker, cloned from the NORMAL extractor's parameters.
    ORBextractor extractor(mFeatures, mScaleFactor, mLevels, mIniThFAST, mMinThFAST);
    while (true)
    {
        int index = -1;
        int cursorAtClaim = 0;
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mScheduleCv.wait(lock, [this] {
                return mStop || (mNextToSchedule < mNumFrames &&
                                 mNextToSchedule < mConsumeCursor + mMaxAhead);
            });
            if (mStop)
            {
                return;
            }
            index = mNextToSchedule;
            ++mNextToSchedule;
            cursorAtClaim = mConsumeCursor;
        }

        // The consumer skips frames served by the INITIALIZATION extractor (which never call Take),
        // jumping mConsumeCursor forward. Such already-passed indices never need extracting, so
        // advance the schedule counter past them cheaply instead of wasting an extraction.
        if (index < cursorAtClaim)
        {
            continue;
        }

        // Heavy work without the lock so workers extract in parallel.
        const cv::Mat image = LoadAndPreprocess(index);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        int monoIndex = 0;
        if (!image.empty())
        {
            // vLapping {0,1000} matches Frame::ExtractORB(0,imGray,0,1000) for the monocular path.
            std::vector<int> lapping = {0, 1000};
            monoIndex = extractor(image, cv::Mat(), keypoints, descriptors, lapping);
        }

        {
            std::unique_lock<std::mutex> lock(mMutex);
            if (index >= mConsumeCursor)  // still relevant (not skipped past by the consumer)
            {
                Entry& entry = mEntries[index];
                entry.keypoints = std::move(keypoints);
                entry.descriptors = std::move(descriptors);
                entry.monoIndex = monoIndex;
                entry.ready = true;
            }
        }
        mReadyCv.notify_all();
    }
}

bool FeaturePrefetcher::Take(const double timestamp, std::vector<cv::KeyPoint>& keypoints,
                             cv::Mat& descriptors, int& monoIndex)
{
    // Find this timestamp at or ahead of the cursor (exact double match; same values flow through).
    int index = mConsumeCursor;
    while (index < mNumFrames && mTimestamps[index] != timestamp)
    {
        ++index;
    }
    if (index >= mNumFrames)
    {
        return false;  // not a prefetched frame -> caller extracts synchronously
    }

    std::unique_lock<std::mutex> lock(mMutex);
    if (index > mConsumeCursor)
    {
        // Jumped past frames served by the ini extractor. Wake blocked workers so they extend the
        // scheduling window to include this frame (otherwise they wait forever and Take deadlocks).
        mConsumeCursor = index;
        mScheduleCv.notify_all();
    }
    mReadyCv.wait(lock, [this, index] {
        const std::unordered_map<int, Entry>::const_iterator found = mEntries.find(index);
        return mStop || (found != mEntries.end() && found->second.ready);
    });
    if (mStop)
    {
        return false;
    }

    const std::unordered_map<int, Entry>::iterator found = mEntries.find(index);
    keypoints = std::move(found->second.keypoints);
    descriptors = std::move(found->second.descriptors);
    monoIndex = found->second.monoIndex;

    // Advance the cursor and drop everything at or below it (frees memory, unblocks workers).
    mConsumeCursor = index + 1;
    for (std::unordered_map<int, Entry>::iterator iter = mEntries.begin(); iter != mEntries.end();)
    {
        if (iter->first < mConsumeCursor)
        {
            iter = mEntries.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
    lock.unlock();
    mScheduleCv.notify_all();
    return true;
}

}  // namespace ORB_SLAM3
