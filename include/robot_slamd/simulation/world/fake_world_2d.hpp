#pragma once

#include "robot_slamd/simulation/core/sim_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robot_slamd {

struct SimLineSegment2D {
    double x1_m = 0.0;
    double y1_m = 0.0;
    double x2_m = 0.0;
    double y2_m = 0.0;
    int id = 0;
};

struct SimRaycastResult {
    bool hit = false;
    double distance_m = 0.0;
    double hit_x_m = 0.0;
    double hit_y_m = 0.0;
    int segment_id = -1;
};

class FakeWorld2D {
public:
    bool add_segment(double x1_m, double y1_m, double x2_m, double y2_m, int id = -1) {
        if (!sim_finite(x1_m) || !sim_finite(y1_m) ||
            !sim_finite(x2_m) || !sim_finite(y2_m)) {
            return false;
        }
        SimLineSegment2D segment;
        segment.x1_m = x1_m;
        segment.y1_m = y1_m;
        segment.x2_m = x2_m;
        segment.y2_m = y2_m;
        segment.id = id >= 0 ? id : next_segment_id_++;
        segments_.push_back(segment);
        return true;
    }

    bool add_axis_aligned_room(double min_x_m,
                               double min_y_m,
                               double max_x_m,
                               double max_y_m) {
        if (!valid_rect(min_x_m, min_y_m, max_x_m, max_y_m)) {
            return false;
        }
        const bool ok1 = add_segment(min_x_m, min_y_m, max_x_m, min_y_m);
        const bool ok2 = add_segment(max_x_m, min_y_m, max_x_m, max_y_m);
        const bool ok3 = add_segment(max_x_m, max_y_m, min_x_m, max_y_m);
        const bool ok4 = add_segment(min_x_m, max_y_m, min_x_m, min_y_m);
        return ok1 && ok2 && ok3 && ok4;
    }

    bool add_axis_aligned_obstacle(double min_x_m,
                                   double min_y_m,
                                   double max_x_m,
                                   double max_y_m) {
        if (!valid_rect(min_x_m, min_y_m, max_x_m, max_y_m)) {
            return false;
        }
        SolidRect rect;
        rect.min_x_m = min_x_m;
        rect.min_y_m = min_y_m;
        rect.max_x_m = max_x_m;
        rect.max_y_m = max_y_m;
        obstacles_.push_back(rect);
        return add_axis_aligned_room(min_x_m, min_y_m, max_x_m, max_y_m);
    }

    SimRaycastResult raycast(double origin_x_m,
                             double origin_y_m,
                             double ray_yaw_rad,
                             double min_range_m,
                             double max_range_m) const {
        SimRaycastResult result;
        result.distance_m = max_range_m;
        result.hit_x_m = origin_x_m + std::cos(ray_yaw_rad) * max_range_m;
        result.hit_y_m = origin_y_m + std::sin(ray_yaw_rad) * max_range_m;
        if (!sim_finite(origin_x_m) || !sim_finite(origin_y_m) ||
            !sim_finite(ray_yaw_rad) || !sim_finite(min_range_m) ||
            !sim_finite(max_range_m) || min_range_m < 0.0 ||
            max_range_m <= min_range_m) {
            result.distance_m = 0.0;
            result.hit_x_m = 0.0;
            result.hit_y_m = 0.0;
            return result;
        }

        const double rx = std::cos(ray_yaw_rad);
        const double ry = std::sin(ray_yaw_rad);
        double best_t = max_range_m;
        int best_id = -1;
        constexpr double eps = 1e-10;
        for (const auto &segment : segments_) {
            const double sx = segment.x2_m - segment.x1_m;
            const double sy = segment.y2_m - segment.y1_m;
            const double qpx = segment.x1_m - origin_x_m;
            const double qpy = segment.y1_m - origin_y_m;
            const double denom = cross(rx, ry, sx, sy);
            if (std::fabs(denom) < eps) {
                continue;
            }
            const double t = cross(qpx, qpy, sx, sy) / denom;
            const double u = cross(qpx, qpy, rx, ry) / denom;
            if (t + eps < min_range_m || t - eps > max_range_m) {
                continue;
            }
            if (u + eps < 0.0 || u - eps > 1.0) {
                continue;
            }
            if (t < best_t) {
                best_t = std::max(0.0, t);
                best_id = segment.id;
            }
        }
        if (best_id >= 0) {
            result.hit = true;
            result.distance_m = best_t;
            result.hit_x_m = origin_x_m + rx * best_t;
            result.hit_y_m = origin_y_m + ry * best_t;
            result.segment_id = best_id;
        }
        return result;
    }

    bool circle_collides(double x_m, double y_m, double radius_m) const {
        if (!sim_finite(x_m) || !sim_finite(y_m) ||
            !sim_finite(radius_m) || radius_m < 0.0) {
            return true;
        }
        for (const auto &rect : obstacles_) {
            if (x_m >= rect.min_x_m - radius_m && x_m <= rect.max_x_m + radius_m &&
                y_m >= rect.min_y_m - radius_m && y_m <= rect.max_y_m + radius_m) {
                return true;
            }
        }
        for (const auto &segment : segments_) {
            if (point_segment_distance(x_m, y_m, segment) <= radius_m) {
                return true;
            }
        }
        return false;
    }

    const std::vector<SimLineSegment2D> &segments() const {
        return segments_;
    }

private:
    struct SolidRect {
        double min_x_m = 0.0;
        double min_y_m = 0.0;
        double max_x_m = 0.0;
        double max_y_m = 0.0;
    };

    static bool valid_rect(double min_x_m, double min_y_m, double max_x_m, double max_y_m) {
        return sim_finite(min_x_m) && sim_finite(min_y_m) &&
               sim_finite(max_x_m) && sim_finite(max_y_m) &&
               max_x_m > min_x_m && max_y_m > min_y_m;
    }

    static double cross(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    static double point_segment_distance(double x_m,
                                         double y_m,
                                         const SimLineSegment2D &segment) {
        const double vx = segment.x2_m - segment.x1_m;
        const double vy = segment.y2_m - segment.y1_m;
        const double wx = x_m - segment.x1_m;
        const double wy = y_m - segment.y1_m;
        const double len2 = vx * vx + vy * vy;
        if (len2 <= 0.0) {
            const double dx = x_m - segment.x1_m;
            const double dy = y_m - segment.y1_m;
            return std::sqrt(dx * dx + dy * dy);
        }
        const double t = sim_clamp((wx * vx + wy * vy) / len2, 0.0, 1.0);
        const double px = segment.x1_m + t * vx;
        const double py = segment.y1_m + t * vy;
        const double dx = x_m - px;
        const double dy = y_m - py;
        return std::sqrt(dx * dx + dy * dy);
    }

    std::vector<SimLineSegment2D> segments_;
    std::vector<SolidRect> obstacles_;
    int next_segment_id_ = 1;
};

} // namespace robot_slamd
