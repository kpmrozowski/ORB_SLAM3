#include "ICAlign.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraModels/GeometricCamera.h"

namespace ORB_SLAM3
{

namespace
{

constexpr double kInvertibilityEpsilon = 1E-10;

// The reductions are summed as a fixed number of contiguous chunks, in ascending chunk order, so the
// result is bit-identical regardless of how many worker threads process the chunks.
constexpr int kNumChunks = 64;

using Vector8f = Eigen::Matrix<float, 8, 1>;
using Matrix8f = Eigen::Matrix<float, 8, 8>;
using Matrix8d = Eigen::Matrix<double, 8, 8>;

enum class SolverState
{
    Success,
    PartialSuccess,
    Failed,
};

enum class InnerIterationState
{
    Success,
    Failed
};

enum class IterationState
{
    EpsilonReached,
    StepAccept,
    StepReject
};

/// Process the flat index range [0,size) as kNumChunks contiguous chunks. `chunk_fn(lo,hi)` returns a
/// chunk's partial; the partials are returned in ascending chunk order so the caller sums them
/// deterministically. Chunks are statically strided across the workers, so the thread count changes
/// only WHICH thread computes a chunk, never the values or their order -> bit-identical output.
template <typename Partial, typename ChunkFn>
std::vector<Partial> run_chunked(const int size, const int num_threads, const Partial& zero,
                                 const ChunkFn& chunk_fn)
{
    std::vector<Partial> partials(kNumChunks, zero);
    const int effective_threads = std::max(1, std::min(num_threads, kNumChunks));

    if (effective_threads == 1)
    {
        for (int chunk = 0; chunk < kNumChunks; ++chunk)
        {
            const int chunk_lo = static_cast<int>((static_cast<long long>(chunk) * size) / kNumChunks);
            const int chunk_hi = static_cast<int>((static_cast<long long>(chunk + 1) * size) / kNumChunks);
            partials[chunk] = chunk_fn(chunk_lo, chunk_hi);
        }
        return partials;
    }

    std::vector<std::thread> workers;
    workers.reserve(effective_threads);
    for (int thread_id = 0; thread_id < effective_threads; ++thread_id)
    {
        workers.emplace_back(
            [&partials, &chunk_fn, size, effective_threads](const int start_chunk)
            {
                for (int chunk = start_chunk; chunk < kNumChunks; chunk += effective_threads)
                {
                    const int chunk_lo = static_cast<int>((static_cast<long long>(chunk) * size) / kNumChunks);
                    const int chunk_hi = static_cast<int>((static_cast<long long>(chunk + 1) * size) / kNumChunks);
                    partials[chunk] = chunk_fn(chunk_lo, chunk_hi);
                }
            },
            thread_id);
    }
    for (std::thread& worker : workers)
    {
        worker.join();
    }
    return partials;
}

struct HomographyWarp
{
    Eigen::Matrix3d homography_ = Eigen::Matrix3d::Identity();

    void update(const Vector8f& increment)
    {
        const double a = increment(0);
        const double b = increment(1);
        const double c = increment(2);
        const double d = increment(3);
        const double e = increment(4);
        const double f = increment(5);
        const double g = increment(6);
        const double h = increment(7);
        const double ap = homography_(0, 0);
        const double bp = homography_(0, 1);
        const double cp = homography_(0, 2);
        const double dp = homography_(1, 0);
        const double ep = homography_(1, 1);
        const double fp = homography_(1, 2);
        const double gp = homography_(2, 0);
        const double hp = homography_(2, 1);

        const double det = f * hp + a * f * hp - c * d * hp + gp * (c - b * f + c * e) - a + b * d - e - a * e - 1;
        if (det * det > kInvertibilityEpsilon)
        {
            homography_(0, 0) = ((d * bp - f * g * bp) + cp * (g - d * h + g * e) + (ap + 1) * (f * h - e - 1)) / det - 1;
            homography_(0, 1) =
                (h * cp + a * h * cp - b * g * cp - bp - a * bp + c * g * bp + b - c * h + b * ap - c * h * ap) / det;
            homography_(0, 2) =
                (f * bp + a * f * bp - c * d * bp + (ap + 1) * (c - b * f + c * e) + cp * (-a + b * d - e - a * e - 1)) /
                det;
            homography_(1, 0) = (fp * (g - d * h + g * e) + d - f * g + d * ep - f * g * ep + dp * (f * h - e - 1)) / det;
            homography_(1, 1) =
                (b * dp - c * h * dp + h * fp + a * h * fp - b * g * fp - a + c * g - ep - a * ep + c * g * ep - 1) /
                    det -
                1;
            homography_(1, 2) = (dp * (c - b * f + c * e) + f + a * f - c * d + f * ep + a * f * ep - c * d * ep +
                                 fp * (-a + b * d - e - a * e - 1)) /
                                det;
            homography_(2, 0) = (d * hp - f * g * hp + g - d * h + g * e + gp * (f * h - e - 1)) / det;
            homography_(2, 1) = (h + a * h - b * g + b * gp - c * h * gp - hp - a * hp + c * g * hp) / det;
        }
    }

    cv::Mat1d get_as_cv() const
    {
        cv::Mat1d homography(3, 3);
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                homography(row, col) = homography_(row, col);
            }
        }
        return homography;
    }

    explicit HomographyWarp(const cv::Mat1d& initial_guess)
    {
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                homography_(row, col) = initial_guess(row, col);
            }
        }
    }

    HomographyWarp() = default;
};

struct Results
{
    float initial_correlation_ = -1.f;
    float final_correlation_ = -1.f;

    HomographyWarp initial_warp_;
    HomographyWarp final_warp_;

    int gauss_iterations_ = -1;
    int gauss_newton_iterations_ = -1;

    SolverState gauss_state = SolverState::Failed;
    SolverState gauss_newton_state = SolverState::Failed;
};

struct ProcessedImage
{
    cv::Mat1f input_image_;
    cv::Mat1b validity_mask_;
    cv::Mat1f warped_image_;
    cv::Mat1b warped_mask_;
    cv::Mat1f buffer_x_, buffer_y_;
    float image_norm_ = 0.f;
    int valid_points_ = 0;

    explicit ProcessedImage(const cv::Mat1b& input_image)
    {
        input_image.convertTo(input_image_, CV_32FC1);
        validity_mask_ = cv::Mat1b::ones(input_image_.rows, input_image_.cols);
    }

    void warp(const HomographyWarp& warp_in)
    {
        const int rows = input_image_.rows;
        const int cols = input_image_.cols;

        if (buffer_x_.empty())
        {
            buffer_x_ = cv::Mat1f(rows, cols);
            buffer_y_ = cv::Mat1f(rows, cols);
        }

        const Eigen::Matrix3f homography = warp_in.homography_.cast<float>();
        for (int row = 0; row < rows; ++row)
        {
            const float row_part_normalization = row * homography(2, 1) + homography(2, 2);
            const float row_part_x = row * homography(0, 1) + homography(0, 2);
            const float row_part_y = row * homography(1, 1) + homography(1, 2);
            for (int col = 0; col < cols; ++col)
            {
                const float normalization_coeff = 1.0f / (col * homography(2, 0) + row_part_normalization);
                buffer_x_(row, col) = (col * homography(0, 0) + row_part_x) * normalization_coeff;
                buffer_y_(row, col) = (col * homography(1, 0) + row_part_y) * normalization_coeff;
            }
        }
        cv::remap(input_image_, warped_image_, buffer_x_, buffer_y_, cv::INTER_LINEAR);
        cv::remap(validity_mask_, warped_mask_, buffer_x_, buffer_y_, cv::INTER_NEAREST);
    }

    void de_mean_and_calc_norm_input_full()
    {
        cv::Scalar img_mean, img_std;
        cv::meanStdDev(input_image_, img_mean, img_std);
        cv::subtract(input_image_, img_mean, input_image_);
        image_norm_ = std::sqrt(input_image_.rows * input_image_.cols * (img_std.val[0]) * (img_std.val[0]));
    }

    void de_mean_and_calc_norm_warped()
    {
        cv::Scalar img_mean, img_std;
        cv::meanStdDev(warped_image_, img_mean, img_std, warped_mask_);
        cv::subtract(warped_image_, img_mean, warped_image_);
        valid_points_ = cv::countNonZero(warped_mask_);
        image_norm_ = std::sqrt(valid_points_ * (img_std.val[0]) * (img_std.val[0]));
    }
};

class StrategyInnerIteration
{
public:
    float lambda_ = 1.0f;

    void accept_step() { lambda_ /= 3.0f; }
    void reject_step() { lambda_ *= 2.0f; }
};

// Newton-phase step acceptance (the lab's non-permissive path: the permissive branch was dead in the
// refine_homography usage, so it is not ported). Keeps the best-so-far warp and stops on an epsilon
// correlation change; any non-improving step hands control to the Gauss-Newton phase.
class StrategyGauss
{
    const float termination_eps_;
    float best_correlation_ = -1.0f;
    HomographyWarp best_warp_;
    float last_correlation_ = -1.0f;

public:
    HomographyWarp view_best_warp() const { return best_warp_; }
    float view_best_correlation() const { return best_correlation_; }

    IterationState accept_step(const float current_estimate, const HomographyWarp& warp_in)
    {
        const float difference = last_correlation_ - current_estimate;
        last_correlation_ = current_estimate;

        const float best_correlation_memory = best_correlation_;
        if (best_correlation_ < current_estimate)
        {
            best_correlation_ = current_estimate;
            best_warp_ = warp_in;
        }

        if (std::abs(difference) < termination_eps_)
        {
            return IterationState::EpsilonReached;
        }
        if (current_estimate > best_correlation_memory)
        {
            return IterationState::StepAccept;
        }
        return IterationState::StepReject;
    }

    StrategyGauss(const float initial_correlation, const HomographyWarp& warp_in, const float termination_eps)
        : termination_eps_(termination_eps),
          best_correlation_(initial_correlation),
          best_warp_(warp_in),
          last_correlation_(initial_correlation)
    {
    }
};

void gradient(const cv::Mat1b& image, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& grad_x,
              Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& grad_y)
{
    const int rows = image.rows;
    const int cols = image.cols;
    for (int row = 1; row < rows - 1; ++row)
    {
        for (int col = 1; col < cols - 1; ++col)
        {
            grad_x(row, col) = 0.5f * (float(image(row, col + 1)) - float(image(row, col - 1)));
            grad_y(row, col) = 0.5f * (float(image(row + 1, col)) - float(image(row - 1, col)));
        }
    }
}

void steepest_descent_images(const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& grad_x,
                             const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& grad_y,
                             const Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& jacobian_x,
                             const Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& jacobian_y,
                             Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& steepest)
{
    const float* grad_x_ptr = grad_x.data();
    const float* grad_y_ptr = grad_y.data();
    const Vector8f* jacobian_x_ptr = jacobian_x.data();
    const Vector8f* jacobian_y_ptr = jacobian_y.data();
    Vector8f* destination = steepest.data();
    const int size = static_cast<int>(steepest.rows() * steepest.cols());
    for (int idx = 0; idx < size; ++idx)
    {
        destination[idx] = grad_x_ptr[idx] * jacobian_x_ptr[idx] + grad_y_ptr[idx] * jacobian_y_ptr[idx];
    }
}

Matrix8f compute_hessian_in_window(const Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& steepest,
                                   const cv::Mat1b& mask, const int num_threads)
{
    const Vector8f* steepest_values = steepest.data();
    const unsigned char* mask_ptr = mask.ptr<unsigned char>();
    const int size = static_cast<int>(steepest.rows() * steepest.cols());

    const std::vector<Matrix8d> partials = run_chunked<Matrix8d>(
        size, num_threads, Matrix8d::Zero(),
        [steepest_values, mask_ptr](const int chunk_lo, const int chunk_hi) -> Matrix8d
        {
            Matrix8d partial = Matrix8d::Zero();
            for (int idx = chunk_lo; idx < chunk_hi; ++idx)
            {
                if (mask_ptr[idx] != 0)
                {
                    partial.selfadjointView<Eigen::Upper>().rankUpdate(steepest_values[idx].cast<double>(), 1.0);
                }
            }
            return partial;
        });

    Matrix8d hessian_upper = Matrix8d::Zero();
    for (const Matrix8d& partial : partials)
    {
        hessian_upper += partial;
    }
    const Matrix8d hessian = hessian_upper.selfadjointView<Eigen::Upper>();
    return hessian.cast<float>();
}

Vector8f difference_vector(const Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& steepest, const cv::Mat1f& warped,
                           const cv::Mat1b& mask, const cv::Mat1f& source, const int num_threads)
{
    const Vector8f* steepest_values = steepest.data();
    const float* warped_ptr = warped.ptr<float>();
    const float* source_ptr = source.ptr<float>();
    const unsigned char* mask_ptr = mask.ptr<unsigned char>();
    const int size = static_cast<int>(steepest.rows() * steepest.cols());

    const std::vector<Vector8f> partials = run_chunked<Vector8f>(
        size, num_threads, Vector8f::Zero(),
        [steepest_values, warped_ptr, source_ptr, mask_ptr](const int chunk_lo, const int chunk_hi) -> Vector8f
        {
            Vector8f partial = Vector8f::Zero();
            for (int idx = chunk_lo; idx < chunk_hi; ++idx)
            {
                const float difference = mask_ptr[idx] * (warped_ptr[idx] - source_ptr[idx]);
                partial += steepest_values[idx] * difference;
            }
            return partial;
        });

    Vector8f difference_vec = Vector8f::Zero();
    for (const Vector8f& partial : partials)
    {
        difference_vec += partial;
    }
    return difference_vec;
}

float multiply_sum_with_mask(const cv::Mat1f& lhs, const cv::Mat1f& rhs, const cv::Mat1b& mask, const int num_threads)
{
    const float* lhs_ptr = lhs.ptr<float>();
    const float* rhs_ptr = rhs.ptr<float>();
    const unsigned char* mask_ptr = mask.ptr<unsigned char>();
    const int size = lhs.rows * lhs.cols;

    const std::vector<float> partials = run_chunked<float>(
        size, num_threads, 0.0f,
        [lhs_ptr, rhs_ptr, mask_ptr](const int chunk_lo, const int chunk_hi) -> float
        {
            float partial = 0.0f;
            for (int idx = chunk_lo; idx < chunk_hi; ++idx)
            {
                partial += lhs_ptr[idx] * rhs_ptr[idx] * mask_ptr[idx];
            }
            return partial;
        });

    float sum = 0.0f;
    for (const float partial : partials)
    {
        sum += partial;
    }
    return sum;
}

Results newton_alignment(const ProcessedImage& source, ProcessedImage& target, const HomographyWarp& input_warp,
                         const Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& steepest, const Matrix8f& hessian,
                         const ICStopCriteria& stop_criteria, const int num_threads)
{
    Results results;

    if (target.image_norm_ <= stop_criteria.deviation_threshold)
    {
        return results;
    }

    const Eigen::LDLT<Matrix8f> ldlt_hessian(hessian);

    results.initial_correlation_ =
        multiply_sum_with_mask(source.input_image_, target.warped_image_, target.warped_mask_, num_threads) /
        (source.image_norm_ * target.image_norm_);

    StrategyGauss strategy(results.initial_correlation_, input_warp, stop_criteria.termination_eps);
    HomographyWarp current_warp = input_warp;

    const HomographyWarp initial_warp = input_warp;
    for (int gauss_iterations = 0; gauss_iterations < stop_criteria.gauss_iterations; ++gauss_iterations)
    {
        const Vector8f difference_vec =
            difference_vector(steepest, target.warped_image_, target.warped_mask_, source.input_image_, num_threads);
        const Vector8f increment = ldlt_hessian.solve(difference_vec);
        current_warp.update(increment);

        target.warp(current_warp);
        target.de_mean_and_calc_norm_warped();

        if (target.image_norm_ < stop_criteria.deviation_threshold)
        {
            results.final_correlation_ = strategy.view_best_correlation();
            results.initial_warp_ = initial_warp;
            results.final_warp_ = strategy.view_best_warp();
            results.gauss_iterations_ = gauss_iterations;
            results.gauss_state = SolverState::PartialSuccess;
            return results;
        }

        const float current_correlation =
            multiply_sum_with_mask(source.input_image_, target.warped_image_, target.warped_mask_, num_threads) /
            (source.image_norm_ * target.image_norm_);

        const IterationState state = strategy.accept_step(current_correlation, current_warp);

        results.final_correlation_ = strategy.view_best_correlation();
        results.initial_warp_ = initial_warp;
        results.final_warp_ = strategy.view_best_warp();
        results.gauss_iterations_ = gauss_iterations;

        if (state == IterationState::EpsilonReached)
        {
            results.gauss_state = SolverState::Success;
            return results;
        }
        if (state == IterationState::StepReject)
        {
            results.gauss_state = SolverState::PartialSuccess;
            return results;
        }
    }

    results.final_correlation_ = strategy.view_best_correlation();
    results.initial_warp_ = initial_warp;
    results.final_warp_ = strategy.view_best_warp();
    results.gauss_iterations_ = stop_criteria.gauss_iterations;
    results.gauss_state = SolverState::PartialSuccess;
    return results;
}

void gauss_newton_alignment(const ProcessedImage& source, ProcessedImage& target, const HomographyWarp& input_warp,
                            const Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor>& steepest, const Matrix8f& hessian,
                            Results& results, const ICStopCriteria& stop_criteria, const int num_threads)
{
    target.warp(input_warp);
    target.de_mean_and_calc_norm_warped();

    InnerIterationState inner_iteration_state = InnerIterationState::Success;

    Matrix8f hessian_lm = hessian;
    Vector8f diagonal = hessian_lm.diagonal();

    StrategyInnerIteration strategy;

    float last_correlation = results.final_correlation_;
    float current_correlation = results.final_correlation_;

    HomographyWarp last_warp = input_warp;
    HomographyWarp best_warp = input_warp;

    for (int gn_iteration = 0; gn_iteration < stop_criteria.gauss_newton_iterations; ++gn_iteration)
    {
        const Vector8f difference_vec =
            difference_vector(steepest, target.warped_image_, target.warped_mask_, source.input_image_, num_threads);

        int inner_iteration = 0;
        for (; inner_iteration < stop_criteria.max_inner_iterations; ++inner_iteration)
        {
            hessian_lm.diagonal() = diagonal * (1.0f + strategy.lambda_);
            const Vector8f increment = hessian_lm.ldlt().solve(difference_vec);

            HomographyWarp estimated_warp = best_warp;
            estimated_warp.update(increment);

            target.warp(estimated_warp);
            target.de_mean_and_calc_norm_warped();

            current_correlation =
                multiply_sum_with_mask(source.input_image_, target.warped_image_, target.warped_mask_, num_threads) /
                (source.image_norm_ * target.image_norm_);

            if (current_correlation > last_correlation)
            {
                inner_iteration_state = InnerIterationState::Success;
                best_warp = estimated_warp;
                strategy.accept_step();
                break;
            }
            inner_iteration_state = InnerIterationState::Failed;
            strategy.reject_step();
        }

        if (inner_iteration_state == InnerIterationState::Failed)
        {
            results.final_correlation_ = current_correlation;
            results.final_warp_ = best_warp;
            results.gauss_newton_iterations_ = gn_iteration;
            results.gauss_newton_state = SolverState::PartialSuccess;
            return;
        }

        const bool epsilon_difference = std::abs(current_correlation - last_correlation) < stop_criteria.termination_eps;
        if (epsilon_difference)
        {
            results.final_correlation_ = current_correlation;
            results.final_warp_ = best_warp;
            results.gauss_newton_iterations_ = gn_iteration;
            results.gauss_newton_state = SolverState::Success;
            return;
        }

        if (current_correlation < last_correlation)
        {
            results.final_correlation_ = current_correlation;
            results.final_warp_ = last_warp;
            results.gauss_newton_iterations_ = gn_iteration;
            results.gauss_newton_state = SolverState::PartialSuccess;
            return;
        }

        last_warp = best_warp;
        last_correlation = current_correlation;
    }

    results.final_correlation_ = current_correlation;
    results.final_warp_ = best_warp;
    results.gauss_newton_iterations_ = stop_criteria.gauss_newton_iterations;
    results.gauss_newton_state = SolverState::PartialSuccess;
}

double consistent_correlation_impl(const cv::Mat1b& source, const cv::Mat1b& target, const cv::Mat1d& warp_cv,
                                   const bool be_consistent)
{
    constexpr int kImageFlags = cv::INTER_LINEAR + cv::WARP_INVERSE_MAP;
    constexpr int kMaskFlags = cv::INTER_NEAREST + cv::WARP_INVERSE_MAP;

    cv::Mat1b target_warped;
    cv::warpPerspective(target, target_warped, warp_cv, target.size(), kImageFlags);

    if (be_consistent)
    {
        cv::Mat1b validity_mask = cv::Mat1b::ones(target.rows, target.cols);
        cv::warpPerspective(validity_mask, validity_mask, warp_cv, validity_mask.size(), kMaskFlags);

        cv::Scalar mean_source, sd_source, mean_target, sd_target;
        cv::meanStdDev(source, mean_source, sd_source, validity_mask);
        cv::meanStdDev(target_warped, mean_target, sd_target, validity_mask);

        const int valid_pixels = cv::countNonZero(validity_mask);
        const float source_scale = std::sqrt(valid_pixels * sd_source[0] * sd_source[0]);
        const float target_scale = std::sqrt(valid_pixels * sd_target[0] * sd_target[0]);

        float correlation = 0.0f;
        for (int row = 0; row < source.rows; ++row)
        {
            for (int col = 0; col < source.cols; ++col)
            {
                if (validity_mask(row, col) == 0)
                {
                    continue;
                }
                const float normalized_source = float(source(row, col)) - mean_source[0];
                const float normalized_target = float(target_warped(row, col)) - mean_target[0];
                correlation += normalized_source * normalized_target;
            }
        }
        return static_cast<double>(correlation / (source_scale * target_scale));
    }

    cv::Scalar mean_source, sd_source, mean_target, sd_target;
    cv::meanStdDev(source, mean_source, sd_source);
    cv::meanStdDev(target_warped, mean_target, sd_target);

    const float source_scale = std::sqrt(source.rows * source.cols * sd_source[0] * sd_source[0]);
    const float target_scale = std::sqrt(source.rows * source.cols * sd_target[0] * sd_target[0]);

    float correlation = 0.0f;
    for (int row = 0; row < source.rows; ++row)
    {
        for (int col = 0; col < source.cols; ++col)
        {
            const float normalized_source = float(source(row, col)) - mean_source[0];
            const float normalized_target = float(target_warped(row, col)) - mean_target[0];
            correlation += normalized_source * normalized_target;
        }
    }
    return static_cast<double>(correlation / (source_scale * target_scale));
}

}  // namespace

bool ICDebugEnabled()
{
    static const bool enabled = getenv("ORB_IC_DEBUG") != nullptr;
    return enabled;
}

int ICThreadCount()
{
    static const int value = []() -> int
    {
        const char* threads = getenv("ORB_IC_THREADS");
        if (threads != nullptr)
        {
            const int parsed = atoi(threads);
            return parsed > 0 ? parsed : 1;
        }
        // Default 1 (deterministic and safe). The chunked reduction is bit-identical for any thread
        // count, so raising ORB_IC_THREADS never perturbs results even under ORB_DETERMINISTIC.
        return 1;
    }();
    return value;
}

double ICCorrelationCoefficient(const cv::Mat1b& source, const cv::Mat1b& target, const cv::Mat1d& warp,
                                const bool be_consistent)
{
    return consistent_correlation_impl(source, target, warp, be_consistent);
}

// --- ICEngine -------------------------------------------------------------------------------------

struct ICEngine::Impl
{
    Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor> jacobian_x_;
    Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor> jacobian_y_;
    Eigen::Matrix<Vector8f, -1, -1, Eigen::RowMajor> steepest_descent_;
    Eigen::Matrix<float, -1, -1, Eigen::RowMajor> grad_x_, grad_y_;
    int num_threads_ = ICThreadCount();

    void init_if_needed(const int rows, const int cols)
    {
        if (rows == jacobian_x_.rows() && cols == jacobian_x_.cols())
        {
            return;
        }
        jacobian_x_.resize(rows, cols);
        jacobian_y_.resize(rows, cols);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                jacobian_x_(row, col) << col, row, 1.0, 0.0, 0.0, 0.0, -col * col, -col * row;
                jacobian_y_(row, col) << 0.0, 0.0, 0.0, col, row, 1.0, -col * row, -row * row;
            }
        }
        steepest_descent_.resize(rows, cols);
        grad_x_ = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>::Zero(rows, cols);
        grad_y_ = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>::Zero(rows, cols);
    }

    std::pair<float, cv::Mat1d> inverse_compositional(const cv::Mat1b& source, const cv::Mat1b& target,
                                                      const cv::Mat1d& warp_cv, const ICStopCriteria& stop_criteria)
    {
        if (source.rows != target.rows || source.cols != target.cols)
        {
            std::ostringstream message;
            message << "ICAlign source/target size mismatch " << source.rows << "x" << source.cols << " vs "
                    << target.rows << "x" << target.cols;
            throw std::logic_error(message.str());
        }
        if (source.rows != jacobian_x_.rows() || source.cols != jacobian_x_.cols())
        {
            throw std::logic_error("ICAlign engine not initialized to the image size");
        }

        gradient(source, grad_x_, grad_y_);

        ProcessedImage target_processed(target);
        target_processed.warp(HomographyWarp(warp_cv));
        target_processed.de_mean_and_calc_norm_warped();

        ProcessedImage source_processed(source);
        source_processed.de_mean_and_calc_norm_input_full();

        steepest_descent_images(grad_x_, grad_y_, jacobian_x_, jacobian_y_, steepest_descent_);
        const Matrix8f hessian =
            compute_hessian_in_window(steepest_descent_, target_processed.warped_mask_, num_threads_);

        Results results = newton_alignment(source_processed, target_processed, HomographyWarp(warp_cv),
                                            steepest_descent_, hessian, stop_criteria, num_threads_);

        if (results.gauss_state == SolverState::Success)
        {
            return {results.final_correlation_, results.final_warp_.get_as_cv()};
        }

        if (results.gauss_state == SolverState::PartialSuccess)
        {
            gauss_newton_alignment(source_processed, target_processed, results.final_warp_, steepest_descent_, hessian,
                                   results, stop_criteria, num_threads_);
            if (results.gauss_newton_state != SolverState::Failed)
            {
                return {results.final_correlation_, results.final_warp_.get_as_cv()};
            }
        }

        return {-1.0f, cv::Mat1d::eye(3, 3)};
    }
};

ICEngine::ICEngine() : impl_(new Impl()) {}
ICEngine::~ICEngine() = default;

std::pair<double, cv::Mat1d> ICEngine::Refine(const cv::Mat1b& source, const cv::Mat1b& target, const cv::Mat1d& seed,
                                              const ICStopCriteria& criteria)
{
    try
    {
        impl_->init_if_needed(source.rows, source.cols);
        cv::Mat1d refined_warp;
        float correlation_ic = -1.0f;
        std::tie(correlation_ic, refined_warp) = impl_->inverse_compositional(source, target, seed, criteria);
        // Re-score against the same consistent ZNCC the seed / ORB candidates use so the gate compares
        // like with like (the solver optimizes a slightly different in-window measure).
        const double correlation_consistent = consistent_correlation_impl(source, target, refined_warp, true);
        (void)correlation_ic;
        return {correlation_consistent, refined_warp};
    }
    catch (const cv::Exception& error)
    {
        if (ICDebugEnabled())
        {
            std::cerr << "[IC] Refine cv::Exception: " << error.what() << std::endl;
        }
    }
    catch (const std::exception& error)
    {
        if (ICDebugEnabled())
        {
            std::cerr << "[IC] Refine std::exception: " << error.what() << std::endl;
        }
    }
    catch (...)
    {
        if (ICDebugEnabled())
        {
            std::cerr << "[IC] Refine non-standard exception" << std::endl;
        }
    }
    return {-1.0, seed.clone()};
}

// --- ICUndistorter --------------------------------------------------------------------------------

std::unique_ptr<ICUndistorter> ICUndistorter::Create(GeometricCamera* camera, const cv::Size& image_size)
{
    if (camera == nullptr || image_size.width <= 0 || image_size.height <= 0)
    {
        return nullptr;
    }

    const unsigned int type = camera->GetType();
    std::unique_ptr<ICUndistorter> undistorter(new ICUndistorter());
    undistorter->image_size_ = image_size;
    undistorter->intrinsic_matrix_ = cv::Matx33d(camera->getParameter(0), 0.0, camera->getParameter(2), 0.0,
                                                 camera->getParameter(1), camera->getParameter(3), 0.0, 0.0, 1.0);

    if (type == GeometricCamera::CAM_FISHEYE)
    {
        undistorter->is_fisheye_ = true;
        undistorter->distortion_coeffs_ =
            (cv::Mat_<double>(4, 1) << camera->getParameter(4), camera->getParameter(5), camera->getParameter(6),
             camera->getParameter(7));
        cv::fisheye::initUndistortRectifyMap(undistorter->intrinsic_matrix_, undistorter->distortion_coeffs_,
                                             cv::Matx33d::eye(), undistorter->intrinsic_matrix_, image_size, CV_16SC2,
                                             undistorter->map1_, undistorter->map2_);
        return undistorter;
    }
    if (type == GeometricCamera::CAM_PINHOLE)
    {
        undistorter->is_fisheye_ = false;
        // A pinhole GeometricCamera carries only [fx,fy,cx,cy]; radial/tangential distortion lives in the
        // frame's mDistCoef, not here, so the P==K rectification is the identity remap. Build it anyway so
        // UndistortImage/UndistortPoints share one code path.
        undistorter->distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        cv::initUndistortRectifyMap(undistorter->intrinsic_matrix_, undistorter->distortion_coeffs_, cv::Matx33d::eye(),
                                    undistorter->intrinsic_matrix_, image_size, CV_16SC2, undistorter->map1_,
                                    undistorter->map2_);
        return undistorter;
    }
    // Unknown camera type -> fail open (caller skips IC).
    return nullptr;
}

cv::Mat1b ICUndistorter::UndistortImage(const cv::Mat1b& distorted) const
{
    cv::Mat1b undistorted;
    cv::remap(distorted, undistorted, map1_, map2_, cv::INTER_LINEAR);
    return undistorted;
}

std::vector<cv::Point2f> ICUndistorter::UndistortPoints(const std::vector<cv::Point2f>& distorted) const
{
    if (distorted.empty())
    {
        return {};
    }
    std::vector<cv::Point2f> undistorted;
    if (is_fisheye_)
    {
        cv::fisheye::undistortPoints(distorted, undistorted, intrinsic_matrix_, distortion_coeffs_, cv::noArray(),
                                     intrinsic_matrix_);
    }
    else
    {
        cv::undistortPoints(distorted, undistorted, intrinsic_matrix_, distortion_coeffs_, cv::noArray(),
                            intrinsic_matrix_);
    }
    return undistorted;
}

Eigen::Matrix3d ICUndistorter::IntrinsicEigen() const
{
    Eigen::Matrix3d intrinsic;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            intrinsic(row, col) = intrinsic_matrix_(row, col);
        }
    }
    return intrinsic;
}

// --- H helpers ------------------------------------------------------------------------------------

cv::Matx33d ICToMatx33(const cv::Mat1d& matrix)
{
    return cv::Matx33d(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(2, 0),
                       matrix(2, 1), matrix(2, 2));
}

double ICTransferError(const cv::Matx33d& homography, const cv::Point2f& from, const cv::Point2f& to)
{
    constexpr double kHomogeneousWeightEps = 1e-12;
    const double from_x = static_cast<double>(from.x);
    const double from_y = static_cast<double>(from.y);
    const double mapped_x = homography(0, 0) * from_x + homography(0, 1) * from_y + homography(0, 2);
    const double mapped_y = homography(1, 0) * from_x + homography(1, 1) * from_y + homography(1, 2);
    const double mapped_w = homography(2, 0) * from_x + homography(2, 1) * from_y + homography(2, 2);
    if (std::abs(mapped_w) <= kHomogeneousWeightEps)
    {
        return std::numeric_limits<double>::infinity();
    }
    const double delta_x = mapped_x / mapped_w - static_cast<double>(to.x);
    const double delta_y = mapped_y / mapped_w - static_cast<double>(to.y);
    return std::hypot(delta_x, delta_y);
}

double ICSymmetricTransferError(const cv::Matx33d& homography, const cv::Matx33d& homography_inverse,
                                const cv::Point2f& point_a, const cv::Point2f& point_b)
{
    const double forward_error = ICTransferError(homography, point_a, point_b);
    const double backward_error = ICTransferError(homography_inverse, point_b, point_a);
    return std::max(forward_error, backward_error);
}

}  // namespace ORB_SLAM3
