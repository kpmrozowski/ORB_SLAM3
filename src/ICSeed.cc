#include "ICSeed.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>

#include <Eigen/Dense>

#include "ICAlign.h"

namespace ORB_SLAM3
{

namespace
{

constexpr double kHomographyScaleEps = 1e-12;
constexpr double kMinForwardNorm = 0.2;
constexpr double kMinTranslationMm = 1e-6;

/// Convert a 3x3 Eigen matrix to a fresh cv::Mat1d.
cv::Mat1d to_cv(const Eigen::Matrix3d& matrix)
{
    cv::Mat1d result(3, 3);
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            result(row, col) = matrix(row, col);
        }
    }
    return result;
}

struct PixelHomography
{
    bool ok = false;
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
};

/// Map a Euclidean homography into the pixel domain (H21 = K * H_e * K^-1) and normalize so H21(2,2)=1.
PixelHomography pixel_homography(const Eigen::Matrix3d& intrinsic, const Eigen::Matrix3d& intrinsic_inverse,
                                 const Eigen::Matrix3d& homography_euclidean)
{
    PixelHomography result;
    const Eigen::Matrix3d raw = intrinsic * homography_euclidean * intrinsic_inverse;
    if (!raw.allFinite() || std::abs(raw(2, 2)) <= kHomographyScaleEps)
    {
        return result;
    }
    const Eigen::Matrix3d normalized = raw / raw(2, 2);
    if (!normalized.allFinite())
    {
        return result;
    }
    result.ok = true;
    result.matrix = normalized;
    return result;
}

/// Faithful cur-from-ref camera translation (mm, cur-cam coords) from a constant-velocity body motion:
/// world = body(ref), velocity = climb*up + speed*forward_horizontal, integrated over dt.
Eigen::Vector3d full_translation_c2_c1(const ICSeedInput& input)
{
    Eigen::Vector3d velocity_world = input.climb_rate_mm_s * input.up_body;
    const Eigen::Vector3d forward_raw = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(input.up_body) * input.up_body;
    if (forward_raw.norm() >= kMinForwardNorm)
    {
        velocity_world += input.horizontal_speed_mm_s * forward_raw.normalized();
    }
    const Eigen::Vector3d position_body_cur_world = velocity_world * input.dt_sec;

    const Eigen::Matrix3d rotation_world_body_cur = input.delta_rotation_ref_from_cur;  // world = body(ref)
    const Eigen::Vector3d translation_world_cam_ref = input.translation_body_cam_mm;    // R_world_cam_ref = R_bc
    const Eigen::Matrix3d rotation_world_cam_cur = rotation_world_body_cur * input.rotation_body_cam;
    const Eigen::Vector3d translation_world_cam_cur =
        rotation_world_body_cur * input.translation_body_cam_mm + position_body_cur_world;

    return rotation_world_cam_cur.transpose() * (translation_world_cam_ref - translation_world_cam_cur);
}

/// Max Euclidean gap between the four image corners mapped through two pixel homographies.
double max_corner_displacement(const Eigen::Matrix3d& homography_a, const Eigen::Matrix3d& homography_b, const int width,
                               const int height)
{
    const std::array<Eigen::Vector3d, 4> corners = {
        Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(width - 1, 0.0, 1.0), Eigen::Vector3d(0.0, height - 1, 1.0),
        Eigen::Vector3d(width - 1, height - 1, 1.0)};
    double max_distance = 0.0;
    for (const Eigen::Vector3d& corner : corners)
    {
        const Eigen::Vector3d mapped_a = homography_a * corner;
        const Eigen::Vector3d mapped_b = homography_b * corner;
        if (std::abs(mapped_a.z()) <= kHomographyScaleEps || std::abs(mapped_b.z()) <= kHomographyScaleEps)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double delta_x = mapped_a.x() / mapped_a.z() - mapped_b.x() / mapped_b.z();
        const double delta_y = mapped_a.y() / mapped_a.z() - mapped_b.y() / mapped_b.z();
        max_distance = std::max(max_distance, std::hypot(delta_x, delta_y));
    }
    return max_distance;
}

}  // namespace

double ICSpeedPriorMps()
{
    static const double value = []() -> double
    {
        const char* speed = getenv("ORB_IC_SPEED_MPS");
        return speed != nullptr ? atof(speed) : 0.0;
    }();
    return value;
}

bool ICSeedPrevEnabled()
{
    static const bool enabled = getenv("ORB_IC_SEED_PREV") != nullptr && atoi(getenv("ORB_IC_SEED_PREV")) != 0;
    return enabled;
}

const char* ICSeedSourceLabel(const ICSeedSource source)
{
    switch (source)
    {
        case ICSeedSource::Full:
            return "full";
        case ICSeedSource::RotationOnly:
            return "rot";
        case ICSeedSource::PrevPair:
            return "prev";
        case ICSeedSource::Failed:
            return "failed";
    }
    return "failed";
}

ICSeedResult ICBuildSeed(const ICSeedInput& input, const cv::Mat1b& undistorted_ref, const cv::Mat1b& undistorted_cur)
{
    ICSeedResult result;
    if (!input.has_rotation)
    {
        return result;  // Failed: no gyro rotation -> caller skips IC (fail open).
    }

    const Eigen::Matrix3d intrinsic = input.intrinsic;
    const Eigen::Matrix3d intrinsic_inverse = intrinsic.inverse();

    // R21 and the rotation-only pixel homography (candidate 2, and the prev-pair "gyro" factor).
    const Eigen::Matrix3d rotation_c2_c1 =
        input.rotation_cam_body * input.delta_rotation_ref_from_cur.transpose() * input.rotation_body_cam;
    const PixelHomography rotation_only = pixel_homography(intrinsic, intrinsic_inverse, rotation_c2_c1);
    if (!rotation_only.ok)
    {
        return result;  // Failed: degenerate rotation homography.
    }
    result.rotation_c2_c1 = rotation_c2_c1;
    result.homography_gyro = rotation_only.matrix;
    result.homography_rotation_only = to_cv(rotation_only.matrix);
    result.zncc_rotation_only =
        ICCorrelationCoefficient(undistorted_ref, undistorted_cur, result.homography_rotation_only, true);

    // Start with rotation-only as the incumbent.
    result.homography = result.homography_rotation_only;
    result.source = ICSeedSource::RotationOnly;
    result.used_seed_zncc = result.zncc_rotation_only;

    // Candidate 1: full seed (gravity/AGL plane + baro-climb translation), metric mm.
    if (input.has_gravity && input.has_agl && input.agl_mm > kMinTranslationMm)
    {
        const Eigen::Vector3d translation_c2_c1 = full_translation_c2_c1(input);
        if (translation_c2_c1.norm() > kMinTranslationMm)
        {
            const Eigen::Vector3d plane_normal_c1 = (input.rotation_cam_body * (-input.up_body)).normalized();
            const Eigen::Matrix3d homography_euclidean =
                rotation_c2_c1 + (translation_c2_c1 * plane_normal_c1.transpose()) / input.agl_mm;
            const PixelHomography full = pixel_homography(intrinsic, intrinsic_inverse, homography_euclidean);
            if (full.ok)
            {
                const cv::Mat1d full_cv = to_cv(full.matrix);
                result.zncc_full = ICCorrelationCoefficient(undistorted_ref, undistorted_cur, full_cv, true);
                if (result.zncc_full > result.used_seed_zncc)
                {
                    result.homography = full_cv;
                    result.source = ICSeedSource::Full;
                    result.used_seed_zncc = result.zncc_full;
                }
            }
        }
    }

    // Candidate 3: previous-pair propagation (TWMM only): strip the previous gyro rotation off the
    // previous refined H and re-apply the current one. Guarded by finiteness and a sane corner move.
    if (input.allow_prev && input.prev_valid)
    {
        const Eigen::Matrix3d prev_gyro_inverse = input.prev_homography_gyro.inverse();
        const Eigen::Matrix3d candidate = rotation_only.matrix * prev_gyro_inverse * input.prev_homography_ic;
        const double corner_move =
            max_corner_displacement(candidate, rotation_only.matrix, input.image_width, input.image_height);
        const double sane_move = static_cast<double>(std::max(input.image_width, input.image_height));
        if (candidate.allFinite() && std::isfinite(corner_move) && corner_move <= sane_move)
        {
            const cv::Mat1d candidate_cv = to_cv(candidate / (std::abs(candidate(2, 2)) > kHomographyScaleEps
                                                                  ? candidate(2, 2)
                                                                  : 1.0));
            result.zncc_prev = ICCorrelationCoefficient(undistorted_ref, undistorted_cur, candidate_cv, true);
            if (result.zncc_prev > result.used_seed_zncc)
            {
                result.homography = candidate_cv;
                result.source = ICSeedSource::PrevPair;
                result.used_seed_zncc = result.zncc_prev;
            }
        }
    }

    return result;
}

ICPoseDelta ICDecomposeHomographyToPose(const cv::Mat1d& homography_ref_to_cur, const Eigen::Matrix3d& intrinsic,
                                        const Eigen::Matrix3d& rotation_c2_c1, const Eigen::Vector3d& plane_normal_c1)
{
    ICPoseDelta delta;

    const double normal_norm = plane_normal_c1.norm();
    if (normal_norm < kMinForwardNorm)
    {
        return delta;  // no usable ground-plane normal (no gravity) -> caller keeps its own prior.
    }
    const Eigen::Vector3d normal_unit = plane_normal_c1 / normal_norm;

    Eigen::Matrix3d homography;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            homography(row, col) = homography_ref_to_cur(row, col);
        }
    }
    if (!homography.allFinite() || std::abs(homography(2, 2)) <= kHomographyScaleEps)
    {
        return delta;
    }

    // Euclidean homography M = K^-1 H K = R21 + (t21/d) n1^T (see the header for the full derivation).
    const Eigen::Matrix3d euclidean = intrinsic.inverse() * homography * intrinsic;
    if (!euclidean.allFinite())
    {
        return delta;
    }
    // Isolate the rank-1 translation term and recover t21/d by right-multiplying with the unit normal.
    const Eigen::Vector3d translation_over_depth = (euclidean - rotation_c2_c1) * normal_unit;
    if (!translation_over_depth.allFinite())
    {
        return delta;
    }

    delta.ok = true;
    delta.rotation_c2_c1 = rotation_c2_c1;
    delta.translation_over_depth = translation_over_depth.norm();
    if (delta.translation_over_depth > kMinTranslationMm)
    {
        delta.unit_translation_c2 = translation_over_depth / delta.translation_over_depth;
    }
    return delta;
}

}  // namespace ORB_SLAM3
