/**
 * ICSeed — IMU/barometer-only seed homography for the dense IC match filter. NEVER derived from ORB
 * (the seed must be independent of the matches it validates).
 *
 * The seed is built in the undistorted P==K pixel domain as H21 (x_cur = H21 * x_ref), the same
 * convention as ORB-SLAM3's two-view H. It offers a CANDIDATE FAMILY and returns the one with the
 * highest consistent ZNCC on the undistorted pair (the lab's "much better median ECC" mechanism):
 *   1. Full   : H = K (R21 + t21 n^T / d) K^-1   (gravity/AGL ground plane + baro-climb translation)
 *   2. Rotation-only : H = K R21 K^-1            (no phantom translation; the lab's workhorse seed)
 *   3. Prev-pair : H = Hgyro(k) Hgyro(k-1)^-1 Hic(k-1)   (TWMM only, ORB_IC_SEED_PREV=1)
 *
 * R21 always comes from gyro preintegration:  R21 = R_cb * dR_ref_from_cur^T * R_bc.
 * The map-point RANSAC plane rung of the plan's ladder is intentionally not built: it is only
 * reachable after a metric map exists, which in the mono pipeline is after IMU init — where the
 * frame-rate gate early-returns and the seed is never requested. The gravity + baro/rfnd AGL plane
 * (metric for both the init and pre-init-track windows) is the only plane path that ever executes.
 */

#ifndef ICSEED_H
#define ICSEED_H

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace ORB_SLAM3
{

/** m/s forward-speed prior for the full seed's horizontal translation (ORB_IC_SPEED_MPS, default 0). */
double ICSpeedPriorMps();
/** true if ORB_IC_SEED_PREV=1 (previous-pair propagation candidate enabled). Read once. */
bool ICSeedPrevEnabled();

enum class ICSeedSource
{
    Full,          ///< gravity/AGL plane + baro-climb translation seed
    RotationOnly,  ///< K R21 K^-1
    PrevPair,      ///< previous-pair propagation
    Failed         ///< no usable gyro rotation -> caller skips IC (fail open)
};

const char* ICSeedSourceLabel(ICSeedSource source);

/** Everything Tracking gathers for one frame pair (ref -> cur), in metric (mm) units where noted. */
struct ICSeedInput
{
    Eigen::Matrix3d intrinsic = Eigen::Matrix3d::Identity();  ///< K (undistorted P==K domain)

    // Extrinsics and gyro rotation (always required for any seed).
    Eigen::Matrix3d rotation_cam_body = Eigen::Matrix3d::Identity();  ///< R_cb (body -> cam)
    Eigen::Matrix3d rotation_body_cam = Eigen::Matrix3d::Identity();  ///< R_bc (cam -> body)
    Eigen::Vector3d translation_body_cam_mm = Eigen::Vector3d::Zero();  ///< t_bc (cam origin in body), mm
    Eigen::Matrix3d delta_rotation_ref_from_cur = Eigen::Matrix3d::Identity();  ///< GetDeltaRotation = R_{bref<-bcur}
    bool has_rotation = false;

    // Gravity direction (mean specific force, body frame) for the ground-plane normal.
    Eigen::Vector3d up_body = Eigen::Vector3d::UnitZ();
    bool has_gravity = false;

    // Metric translation ingredients (baro climb + forward-speed prior), all mm / mm-per-second.
    double climb_rate_mm_s = 0.0;
    double horizontal_speed_mm_s = 0.0;
    double dt_sec = 0.0;
    double agl_mm = 0.0;
    bool has_agl = false;

    // Previous-pair propagation (TWMM only).
    bool allow_prev = false;
    bool prev_valid = false;
    Eigen::Matrix3d prev_homography_gyro = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d prev_homography_ic = Eigen::Matrix3d::Identity();

    int image_width = 0;
    int image_height = 0;
};

struct ICSeedResult
{
    cv::Mat1d homography = cv::Mat1d::eye(3, 3);                ///< the CHOSEN seed (ref -> cur)
    cv::Mat1d homography_rotation_only = cv::Mat1d::eye(3, 3);  ///< K R21 K^-1 (diagnostic / prev-pair)
    ICSeedSource source = ICSeedSource::Failed;
    double used_seed_zncc = -1.0;  ///< consistent ZNCC of the chosen seed on the undistorted pair
    double zncc_full = -1.0;
    double zncc_rotation_only = -1.0;
    double zncc_prev = -1.0;
    Eigen::Matrix3d rotation_c2_c1 = Eigen::Matrix3d::Identity();  ///< R21
    Eigen::Matrix3d homography_gyro = Eigen::Matrix3d::Identity();  ///< K R21 K^-1 (for the next pair's prev term)
};

/** Build the candidate family and return the best-ZNCC seed. `undistorted_ref` / `undistorted_cur`
 *  are the full-res undistorted (P==K) images; ZNCC scores rank the candidates. Never throws. */
ICSeedResult ICBuildSeed(const ICSeedInput& input, const cv::Mat1b& undistorted_ref,
                         const cv::Mat1b& undistorted_cur);

}  // namespace ORB_SLAM3

#endif  // ICSEED_H
