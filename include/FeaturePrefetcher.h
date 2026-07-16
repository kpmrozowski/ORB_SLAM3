/**
 * FeaturePrefetcher — env-gated, multi-threaded ORB feature pre-extraction for offline runs.
 *
 * ORB_PREFETCH=<N|all>   enable prefetching with N worker threads ("all" =
 *                        std::thread::hardware_concurrency()). Default (unset) = OFF, behaviour
 *                        identical to upstream (every Frame extracts synchronously).
 *
 * Idea: ORB feature EXTRACTION (image -> keypoints+descriptors) depends only on the image and the
 * extractor parameters, not on live map state, so it can be precomputed. Worker threads read the
 * upcoming images, run the SAME preprocessing as Tracking::GrabImageMonocular (grayscale
 * conversion + optional ORB_CLAHE) and the SAME ORBextractor configuration as the NORMAL extractor,
 * and cache the result keyed by frame index. When the tracker reaches that frame,
 * Frame::ExtractORB consumes the cached result instead of extracting inline. ORB-SLAM3's MATCHING
 * (SearchByProjection etc.) needs live map state and is NOT precomputed.
 *
 * Only the NORMAL extractor is served: the INITIALIZATION extractor (mpIniORBextractor, 5x
 * features, used pre-init and after resets) always extracts synchronously, so cache entries built
 * for the normal extractor are never mixed up with it (Frame checks the extractor pointer).
 *
 * Correctness note: on a cache hit ORBextractor::ComputePyramid never runs for that frame, so
 * mpORBextractorLeft->mvImagePyramid is stale. For the MONOCULAR path this is safe — that pyramid
 * is only read by Frame::ComputeStereoMatches (stereo only), never by the monocular pipeline.
 */

#ifndef FEATUREPREFETCHER_H
#define FEATUREPREFETCHER_H

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

class ORBextractor;

class FeaturePrefetcher
{
public:
    /**
     * @param imagePaths        image files in the exact order the tracker will process them.
     * @param timestamps        matching timestamps (same double values the example feeds in).
     * @param normalExtractor   the NORMAL ORBextractor whose parameters are cloned and whose
     *                          pointer identity gates cache use (non-owning).
     * @param numWorkers        number of worker threads (>= 1).
     * @param rgb               mbRGB flag, so grayscale conversion matches GrabImageMonocular.
     */
    FeaturePrefetcher(const std::vector<std::string>& imagePaths, const std::vector<double>& timestamps,
                      const ORBextractor* normalExtractor, const int numWorkers, const bool rgb);

    ~FeaturePrefetcher();

    FeaturePrefetcher(const FeaturePrefetcher&) = delete;
    FeaturePrefetcher& operator=(const FeaturePrefetcher&) = delete;

    /**
     * Consume the cached extraction for @p timestamp. Timestamps must arrive in strictly
     * increasing order (matching the processing order). Blocks until the entry is ready.
     * @return true and moves keypoints/descriptors/monoIndex out on a cache hit; false if the
     *         timestamp is not in the registered list (caller extracts synchronously).
     */
    bool Take(const double timestamp, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
              int& monoIndex);

    /** The NORMAL extractor this prefetcher serves (non-owning). */
    const ORBextractor* NormalExtractor() const { return mNormalExtractor; }

    /** Process-wide non-owning registry so Frame::ExtractORB can find the active prefetcher. */
    static void Register(FeaturePrefetcher* prefetcher) { sRegistered = prefetcher; }
    static FeaturePrefetcher* Registered() { return sRegistered; }

private:
    struct Entry
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        int monoIndex = 0;
        bool ready = false;
    };

    void WorkerLoop();
    cv::Mat LoadAndPreprocess(const int index) const;

    const std::vector<std::string> mImagePaths;
    const std::vector<double> mTimestamps;
    const ORBextractor* const mNormalExtractor;  // non-owning
    const int mNumFrames;
    const int mMaxAhead;
    const bool mRGB;

    // Cloned NORMAL extractor parameters.
    const int mFeatures;
    const float mScaleFactor;
    const int mLevels;
    const int mIniThFAST;
    const int mMinThFAST;

    // CLAHE parameters (same envs as Tracking::GrabImageMonocular); 0 clip = disabled.
    const float mClaheClip;
    const int mClaheTile;

    mutable std::mutex mMutex;
    std::condition_variable mReadyCv;     // consumer waits, workers notify on a stored entry
    std::condition_variable mScheduleCv;  // workers wait at the ahead-bound, consumer notifies
    std::unordered_map<int, Entry> mEntries;
    int mConsumeCursor = 0;    // next index expected by Take(); entries below it are dropped
    int mNextToSchedule = 0;   // next index a worker will claim
    bool mStop = false;
    std::vector<std::thread> mWorkers;

    static FeaturePrefetcher* sRegistered;
};

}  // namespace ORB_SLAM3

#endif  // FEATUREPREFETCHER_H
