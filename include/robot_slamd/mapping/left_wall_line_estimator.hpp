#pragma once

#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace robot_slamd {

struct LeftWallPoint {
    double x_m = 0.0;
    double y_m = 0.0;
    double timestamp_s = 0.0;
    double confidence = 0.0;
    std::size_t cycle_index = 0;
    std::uint64_t map_revision = 0;
    std::uint64_t frame_transform_epoch = 0;
    std::uint64_t wall_segment_id = 1;
};

struct LeftWallLineEstimatorConfig {
    std::size_t window_max_points = 10;
    std::size_t min_fit_points = 5;
    double min_baseline_m = 0.05;
    double max_fit_rms_m = 0.03;
    double outlier_min_threshold_m = 0.02;
    double outlier_mad_scale = 3.0;
};

struct LeftWallModel {
    bool valid = false;
    double wall_heading_rad = 0.0;
    double signed_base_to_wall_distance_m = 0.0;
    double rms_residual_m = 0.0;
    double baseline_m = 0.0;
    std::size_t input_point_count = 0;
    std::size_t inlier_point_count = 0;
    std::uint64_t frame_transform_epoch = 0;
    std::uint64_t wall_segment_id = 1;
    std::string reason = "not_fitted";
};

inline bool left_wall_line_estimator_config_valid(
    const LeftWallLineEstimatorConfig &config) {
    return config.window_max_points > 0 &&
           config.min_fit_points >= 2 &&
           config.min_fit_points <= config.window_max_points &&
           std::isfinite(config.min_baseline_m) && config.min_baseline_m > 0.0 &&
           std::isfinite(config.max_fit_rms_m) && config.max_fit_rms_m >= 0.0 &&
           std::isfinite(config.outlier_min_threshold_m) &&
               config.outlier_min_threshold_m >= 0.0 &&
           std::isfinite(config.outlier_mad_scale) &&
               config.outlier_mad_scale > 0.0;
}

class LeftWallLineEstimator final {
public:
    explicit LeftWallLineEstimator(LeftWallLineEstimatorConfig config = {})
        : config_(config) {}

    LeftWallModel fit(const std::vector<LeftWallPoint> &points,
                      const RobotPose2D &robot_pose,
                      std::uint64_t frame_transform_epoch) const {
        LeftWallModel result;
        result.frame_transform_epoch = frame_transform_epoch;
        if (!left_wall_line_estimator_config_valid(config_)) {
            result.reason = "invalid_estimator_config";
            return result;
        }
        if (!sparse_slam_pose_finite(robot_pose)) {
            result.reason = "robot_pose_non_finite";
            return result;
        }

        std::vector<LeftWallPoint> finite;
        finite.reserve(std::min(config_.window_max_points, points.size()));
        std::uint64_t segment_id = 0;
        for (const auto &point : points) {
            if (finite.size() >= config_.window_max_points) break;
            if (point.frame_transform_epoch != frame_transform_epoch ||
                (segment_id != 0 && point.wall_segment_id != segment_id) ||
                !std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
                !std::isfinite(point.timestamp_s) ||
                !std::isfinite(point.confidence)) {
                continue;
            }
            if (segment_id == 0) segment_id = point.wall_segment_id;
            finite.push_back(point);
        }
        result.input_point_count = finite.size();
        result.wall_segment_id = segment_id == 0 ? 1 : segment_id;
        if (finite.size() < config_.min_fit_points) {
            result.reason = "insufficient_fit_points";
            return result;
        }

        const auto initial = pca(finite);
        if (!initial.ok) {
            result.reason = initial.reason;
            return result;
        }
        std::vector<double> residuals;
        residuals.reserve(finite.size());
        for (const auto &point : finite) {
            residuals.push_back(std::fabs(
                (point.x_m - initial.cx) * initial.nx +
                (point.y_m - initial.cy) * initial.ny));
        }
        const double median = median_of(residuals);
        std::vector<double> deviations;
        deviations.reserve(residuals.size());
        for (const double residual : residuals) {
            deviations.push_back(std::fabs(residual - median));
        }
        const double mad = median_of(deviations);
        const double threshold = std::max(
            config_.outlier_min_threshold_m,
            config_.outlier_mad_scale * 1.4826 * mad);

        std::vector<LeftWallPoint> inliers;
        inliers.reserve(finite.size());
        for (std::size_t index = 0; index < finite.size(); ++index) {
            if (residuals[index] <= threshold + 1e-12) {
                inliers.push_back(finite[index]);
            }
        }
        result.inlier_point_count = inliers.size();
        if (inliers.size() < config_.min_fit_points) {
            result.reason = "insufficient_inliers";
            return result;
        }

        const auto refined = pca(inliers);
        if (!refined.ok) {
            result.reason = refined.reason;
            return result;
        }
        double tx = refined.tx;
        double ty = refined.ty;
        const double forward_x = std::cos(robot_pose.yaw_rad);
        const double forward_y = std::sin(robot_pose.yaw_rad);
        if (tx * forward_x + ty * forward_y < 0.0) {
            tx = -tx;
            ty = -ty;
        }
        const double wall_heading = std::atan2(ty, tx);
        const double nx = -std::sin(wall_heading);
        const double ny = std::cos(wall_heading);
        double min_projection = std::numeric_limits<double>::infinity();
        double max_projection = -std::numeric_limits<double>::infinity();
        double squared_residual = 0.0;
        for (const auto &point : inliers) {
            const double dx = point.x_m - refined.cx;
            const double dy = point.y_m - refined.cy;
            const double projection = dx * tx + dy * ty;
            min_projection = std::min(min_projection, projection);
            max_projection = std::max(max_projection, projection);
            const double residual = dx * nx + dy * ny;
            squared_residual += residual * residual;
        }
        result.baseline_m = max_projection - min_projection;
        result.rms_residual_m = std::sqrt(
            squared_residual / static_cast<double>(inliers.size()));
        result.wall_heading_rad = sparse_slam_normalize_yaw(wall_heading);
        result.signed_base_to_wall_distance_m =
            nx * (refined.cx - robot_pose.x_m) +
            ny * (refined.cy - robot_pose.y_m);
        if (!std::isfinite(result.baseline_m) ||
            !std::isfinite(result.rms_residual_m) ||
            !std::isfinite(result.signed_base_to_wall_distance_m) ||
            result.baseline_m < config_.min_baseline_m) {
            result.reason = "fit_baseline_too_short";
            return result;
        }
        if (result.rms_residual_m > config_.max_fit_rms_m + 1e-12) {
            result.reason = "fit_rms_exceeded";
            return result;
        }
        if (!(result.signed_base_to_wall_distance_m > 0.0)) {
            result.reason = "wall_not_on_left";
            return result;
        }
        result.valid = true;
        result.reason = "fit_valid";
        return result;
    }

    const LeftWallLineEstimatorConfig &config() const { return config_; }

private:
    struct PcaResult {
        bool ok = false;
        double cx = 0.0;
        double cy = 0.0;
        double tx = 0.0;
        double ty = 0.0;
        double nx = 0.0;
        double ny = 0.0;
        std::string reason = "pca_not_run";
    };

    static PcaResult pca(const std::vector<LeftWallPoint> &points) {
        PcaResult result;
        if (points.size() < 2) {
            result.reason = "pca_insufficient_points";
            return result;
        }
        for (const auto &point : points) {
            result.cx += point.x_m;
            result.cy += point.y_m;
        }
        result.cx /= static_cast<double>(points.size());
        result.cy /= static_cast<double>(points.size());
        double cxx = 0.0;
        double cxy = 0.0;
        double cyy = 0.0;
        for (const auto &point : points) {
            const double dx = point.x_m - result.cx;
            const double dy = point.y_m - result.cy;
            cxx += dx * dx;
            cxy += dx * dy;
            cyy += dy * dy;
        }
        const double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
        result.tx = std::cos(theta);
        result.ty = std::sin(theta);
        result.nx = -result.ty;
        result.ny = result.tx;
        const double trace = cxx + cyy;
        if (!std::isfinite(trace) || trace <= 1e-16 ||
            !std::isfinite(result.cx) || !std::isfinite(result.cy)) {
            result.reason = "pca_degenerate";
            return result;
        }
        result.ok = true;
        result.reason = "pca_valid";
        return result;
    }

    static double median_of(std::vector<double> values) {
        if (values.empty()) return 0.0;
        const auto middle = values.begin() + values.size() / 2U;
        std::nth_element(values.begin(), middle, values.end());
        if (values.size() % 2U != 0U) return *middle;
        const auto lower = std::max_element(values.begin(), middle);
        return 0.5 * (*lower + *middle);
    }

    LeftWallLineEstimatorConfig config_;
};

} // namespace robot_slamd
