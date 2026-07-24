/**
 * ICAlign — dense inverse-compositional homography refinement, ported from the radiolus
 * photometric-alignment lab (corr::Correlation / correlation_coefficient) into the ORB-SLAM3
 * fork as an env-gated ORB false-positive-match filter.
 *
 * Port notes (fork is C++14, no OpenMP, -DEIGEN_DONT_VECTORIZE):
 *  - rlog/io-debug stripped; diagnostics go to stderr under ORB_IC_DEBUG.
 *  - std::format -> std::ostringstream.
 *  - Eigen::Vector<float,8> -> Eigen::Matrix<float,8,1>.
 *  - the OpenMP reductions (hessian, difference vector, masked correlation sum) are replaced by a
 *    FIXED-64-CHUNK std::thread reduction: partials are summed in ascending chunk order, so the
 *    result is bit-identical for ANY thread count (ORB_IC_THREADS; default 1 under ORB_DETERMINISTIC).
 *  - full-resolution only (no image pyramid / reduction).
 *
 * Homography convention: the returned warp maps SOURCE pixel -> TARGET pixel, i.e. x_target =
 * H * x_source (same as cv::findHomography(src,dst) and ORB-SLAM3's H21 in the undistorted P=K
 * pixel domain). Refine() never throws — cv::Exception and std::logic_error are caught and turned
 * into a fail-open (-1 correlation, seed returned unchanged).
 */

#ifndef ICALIGN_H
#define ICALIGN_H

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace ORB_SLAM3
{

class GeometricCamera;

/** true if ORB_IC_DEBUG is set (read once). */
bool ICDebugEnabled();

/** Worker-thread count for the chunked reductions: ORB_IC_THREADS, forced to 1 under
 *  ORB_DETERMINISTIC unless ORB_IC_THREADS is explicitly set. Read once. */
int ICThreadCount();

/** Solver budget for one inverse-compositional refinement (mirrors corr::StopCriteria). */
struct ICStopCriteria
{
    float deviation_threshold = 0.00001f;
    float termination_eps = 1e-6f;
    int gauss_iterations = 30;
    int gauss_newton_iterations = 200;
    int max_inner_iterations = 1;
};

/** Consistent (be_consistent=true, mask-aware) or plain ZNCC of `source` against `target` inversely
 *  warped by `warp` (source px -> target px). Matches corr::correlation_coefficient semantics. */
double ICCorrelationCoefficient(const cv::Mat1b& source, const cv::Mat1b& target, const cv::Mat1d& warp,
                                bool be_consistent);

/** Dense inverse-compositional homography refinement engine. Holds precomputed jacobian / steepest
 *  descent caches (~16 MB at 450x338); keep ONE instance per tracker and reuse it. */
class ICEngine
{
public:
    ICEngine();
    ~ICEngine();

    ICEngine(const ICEngine&) = delete;
    ICEngine& operator=(const ICEngine&) = delete;

    /** Refine `seed` (source->target px) by dense photometric alignment on the undistorted pair.
     *  Returns {refined consistent ZNCC, refined 3x3 double warp}. HONEST STORAGE: the refined warp
     *  is returned even when the ZNCC is low (the caller gates on the score, not the matrix). On any
     *  internal failure returns {-1.0, seed.clone()} — never throws. */
    std::pair<double, cv::Mat1d> Refine(const cv::Mat1b& source, const cv::Mat1b& target, const cv::Mat1d& seed,
                                        const ICStopCriteria& criteria);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/** Fisheye (Kannala-Brandt, CAM_FISHEYE) / pinhole (CAM_PINHOLE) rectifier to P == K (same intrinsics,
 *  same image size). The remap tables (CV_16SC2) are built once; Create() returns nullptr for an
 *  unknown camera type so the caller can fail open. */
class ICUndistorter
{
public:
    static std::unique_ptr<ICUndistorter> Create(GeometricCamera* camera, const cv::Size& image_size);

    cv::Mat1b UndistortImage(const cv::Mat1b& distorted) const;
    std::vector<cv::Point2f> UndistortPoints(const std::vector<cv::Point2f>& distorted) const;

    const cv::Matx33d& IntrinsicMatrix() const { return intrinsic_matrix_; }
    Eigen::Matrix3d IntrinsicEigen() const;

private:
    ICUndistorter() = default;

    bool is_fisheye_ = true;
    cv::Mat map1_;
    cv::Mat map2_;
    cv::Matx33d intrinsic_matrix_ = cv::Matx33d::eye();
    cv::Mat distortion_coeffs_;
    cv::Size image_size_;
};

/** Project `from` through `homography` (homogeneous, w-divided) and return the Euclidean distance to
 *  `to`. A degenerate homogeneous weight yields +inf so the caller treats the match as failed. */
double ICTransferError(const cv::Matx33d& homography, const cv::Point2f& from, const cv::Point2f& to);

/** max(forward, backward) transfer error against a homography and its inverse (symmetric gate). */
double ICSymmetricTransferError(const cv::Matx33d& homography, const cv::Matx33d& homography_inverse,
                                const cv::Point2f& point_a, const cv::Point2f& point_b);

/** 3x3 cv::Mat1d -> cv::Matx33d for fast repeated point transfer. */
cv::Matx33d ICToMatx33(const cv::Mat1d& matrix);

}  // namespace ORB_SLAM3

#endif  // ICALIGN_H
