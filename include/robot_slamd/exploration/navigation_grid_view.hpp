#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_grid_types.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_slamd {

enum class NavigationCellClass {
    Unknown,
    Free,
    Occupied,
    Uncertain,
    InflatedOccupied
};

struct NavigationGridViewConfig {
    double min_x_m = -2.0;
    double max_x_m = 2.0;
    double min_y_m = -2.0;
    double max_y_m = 2.0;
    double robot_radius_m = 0.08;
    double safety_margin_m = 0.04;
    std::size_t maximum_cells = 200000;
};

struct NavigationGridViewBuildResult {
    bool ok = false;
    std::string reason;
};

class NavigationGridView {
public:
    NavigationGridViewBuildResult build(
        const SparseOccupancyGridSnapshot &snapshot,
        const NavigationGridViewConfig &config) {
        clear();
        if (!snapshot.valid() || !std::isfinite(snapshot.resolution_m()) ||
            snapshot.resolution_m() <= 0.0 || !valid_config(config)) {
            return {false, "navigation_grid_invalid_input"};
        }
        resolution_m_ = snapshot.resolution_m();
        min_key_ = sparse_grid_cell_for_world(
            config.min_x_m, config.min_y_m, resolution_m_);
        const auto max_key = sparse_grid_cell_for_world(
            std::nextafter(config.max_x_m, config.min_x_m),
            std::nextafter(config.max_y_m, config.min_y_m), resolution_m_);
        if (max_key.x < min_key_.x || max_key.y < min_key_.y) {
            return {false, "navigation_grid_invalid_bounds"};
        }
        const std::int64_t width64 =
            static_cast<std::int64_t>(max_key.x) - min_key_.x + 1;
        const std::int64_t height64 =
            static_cast<std::int64_t>(max_key.y) - min_key_.y + 1;
        if (width64 <= 0 || height64 <= 0 ||
            static_cast<std::uint64_t>(width64) > config.maximum_cells ||
            static_cast<std::uint64_t>(height64) > config.maximum_cells ||
            static_cast<std::uint64_t>(width64) *
                    static_cast<std::uint64_t>(height64) >
                config.maximum_cells) {
            return {false, "navigation_grid_capacity_exceeded"};
        }
        width_ = static_cast<std::size_t>(width64);
        height_ = static_cast<std::size_t>(height64);
        cells_.assign(width_ * height_, NavigationCellClass::Unknown);
        for (std::size_t y = 0; y < height_; ++y) {
            for (std::size_t x = 0; x < width_; ++x) {
                const auto query = snapshot.query_cell(global_key(x, y));
                if (!query.ok) {
                    clear();
                    return {false, "navigation_grid_snapshot_query_failed"};
                }
                cells_[index(x, y)] = convert(query.classification);
            }
        }
        const int inflation_cells = static_cast<int>(std::ceil(
            (config.robot_radius_m + config.safety_margin_m) / resolution_m_));
        const auto original = cells_;
        for (std::size_t y = 0; y < height_; ++y) {
            for (std::size_t x = 0; x < width_; ++x) {
                if (original[index(x, y)] != NavigationCellClass::Occupied) {
                    continue;
                }
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
                            continue;
                        }
                        const auto nx = static_cast<std::int64_t>(x) + dx;
                        const auto ny = static_cast<std::int64_t>(y) + dy;
                        if (nx < 0 || ny < 0 ||
                            nx >= static_cast<std::int64_t>(width_) ||
                            ny >= static_cast<std::int64_t>(height_)) {
                            continue;
                        }
                        auto &cell = cells_[index(static_cast<std::size_t>(nx),
                                                  static_cast<std::size_t>(ny))];
                        if (cell != NavigationCellClass::Occupied) {
                            cell = NavigationCellClass::InflatedOccupied;
                        }
                    }
                }
            }
        }
        valid_ = true;
        return {true, "navigation_grid_ready"};
    }

    bool valid() const { return valid_; }
    std::size_t width() const { return width_; }
    std::size_t height() const { return height_; }
    std::size_t cell_count() const { return cells_.size(); }
    double resolution_m() const { return resolution_m_; }

    bool contains(const SparseGridCellKey &key) const {
        return valid_ && key.x >= min_key_.x && key.y >= min_key_.y &&
               static_cast<std::int64_t>(key.x) <
                   static_cast<std::int64_t>(min_key_.x) +
                       static_cast<std::int64_t>(width_) &&
               static_cast<std::int64_t>(key.y) <
                   static_cast<std::int64_t>(min_key_.y) +
                       static_cast<std::int64_t>(height_);
    }

    NavigationCellClass cell(const SparseGridCellKey &key) const {
        if (!contains(key)) return NavigationCellClass::Unknown;
        return cells_[index(static_cast<std::size_t>(key.x - min_key_.x),
                            static_cast<std::size_t>(key.y - min_key_.y))];
    }

    bool traversable(const SparseGridCellKey &key) const {
        return cell(key) == NavigationCellClass::Free;
    }

    std::optional<SparseGridCellKey> nearest_traversable(
        const SparseGridCellKey &origin, int maximum_manhattan_radius) const {
        if (!valid_ || maximum_manhattan_radius < 0) return std::nullopt;
        if (traversable(origin)) return origin;
        for (int radius = 1; radius <= maximum_manhattan_radius; ++radius) {
            for (int dx = -radius; dx <= radius; ++dx) {
                const int dy = radius - std::abs(dx);
                const SparseGridCellKey first{origin.x + dx, origin.y - dy};
                if (traversable(first)) return first;
                if (dy != 0) {
                    const SparseGridCellKey second{origin.x + dx, origin.y + dy};
                    if (traversable(second)) return second;
                }
            }
        }
        return std::nullopt;
    }

    SparseGridCellKey world_to_cell(double x_m, double y_m) const {
        return sparse_grid_cell_for_world(x_m, y_m, resolution_m_);
    }

    RobotPose2D cell_center(const SparseGridCellKey &key) const {
        RobotPose2D out;
        out.x_m = (static_cast<double>(key.x) + 0.5) * resolution_m_;
        out.y_m = (static_cast<double>(key.y) + 0.5) * resolution_m_;
        return out;
    }

    SparseGridCellKey global_key(std::size_t x, std::size_t y) const {
        return {static_cast<std::int32_t>(min_key_.x +
                                         static_cast<std::int64_t>(x)),
                static_cast<std::int32_t>(min_key_.y +
                                         static_cast<std::int64_t>(y))};
    }

private:
    static bool valid_config(const NavigationGridViewConfig &config) {
        return std::isfinite(config.min_x_m) &&
               std::isfinite(config.max_x_m) &&
               std::isfinite(config.min_y_m) &&
               std::isfinite(config.max_y_m) &&
               config.max_x_m > config.min_x_m &&
               config.max_y_m > config.min_y_m &&
               std::isfinite(config.robot_radius_m) &&
               config.robot_radius_m >= 0.0 &&
               std::isfinite(config.safety_margin_m) &&
               config.safety_margin_m >= 0.0 && config.maximum_cells > 0;
    }

    static NavigationCellClass convert(SparseReferenceCellClass value) {
        switch (value) {
        case SparseReferenceCellClass::Free: return NavigationCellClass::Free;
        case SparseReferenceCellClass::Occupied: return NavigationCellClass::Occupied;
        case SparseReferenceCellClass::Uncertain: return NavigationCellClass::Uncertain;
        case SparseReferenceCellClass::Unknown: return NavigationCellClass::Unknown;
        }
        return NavigationCellClass::Unknown;
    }

    std::size_t index(std::size_t x, std::size_t y) const {
        return y * width_ + x;
    }

    void clear() {
        valid_ = false;
        width_ = height_ = 0;
        resolution_m_ = 0.0;
        min_key_ = {};
        cells_.clear();
    }

    bool valid_ = false;
    std::size_t width_ = 0;
    std::size_t height_ = 0;
    double resolution_m_ = 0.0;
    SparseGridCellKey min_key_;
    std::vector<NavigationCellClass> cells_;
};

} // namespace robot_slamd
