#pragma once

#include "robot_slamd/mapping/sparse_tof/scalar_tof_return_kind.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_grid_ray_traversal.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_ray_observation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace robot_slamd {

class SparseOccupancyGrid {
public:
    explicit SparseOccupancyGrid(SparseOccupancyGridConfig config = {})
        : config_(config) {}

    bool valid() const {
        return sparse_grid_config_valid(config_);
    }

    const SparseOccupancyGridConfig &config() const {
        return config_;
    }

    SparseGridCellKey cell_for_world(double x_m, double y_m) const {
        return sparse_grid_cell_for_world(x_m, y_m, config_.resolution_m);
    }

    SparseOccupancyUpdateStats apply_observations(
        const std::vector<SparseTofRayObservation> &observations) {
        SparseOccupancyUpdateStats stats;
        if (!valid()) {
            stats.fault = SparseOccupancyGridFault::InvalidConfig;
            stats.message = "sparse_grid_invalid_config";
            return stats;
        }

        std::map<SparseGridCellKey, bool> candidate;
        for (const auto &obs : observations) {
            if (obs.return_kind == ScalarTofReturnKind::Invalid ||
                obs.return_kind == ScalarTofReturnKind::Unspecified ||
                !obs.usable_for_mapping ||
                !obs.protocol_valid ||
                !obs.synchronized) {
                stats.invalid_ray_count++;
                continue;
            }

            const bool hit = obs.return_kind == ScalarTofReturnKind::Hit;
            const bool no_return = obs.return_kind == ScalarTofReturnKind::NoReturn;
            if (!hit && !no_return) {
                stats.invalid_ray_count++;
                continue;
            }

            const double range_m = hit ? obs.measured_range_m
                                       : obs.free_space_range_m;
            auto traversal = sparse_grid_traverse_ray(
                obs.sensor_origin_x_m,
                obs.sensor_origin_y_m,
                obs.ray_yaw_rad,
                std::min(range_m, config_.maximum_mapping_range_m),
                config_,
                true);
            if (!traversal.ok) {
                stats.fault = traversal.fault;
                stats.message = traversal.message;
                return stats;
            }
            if (traversal.cells.empty()) {
                stats.invalid_ray_count++;
                continue;
            }

            stats.valid_ray_count++;
            if (hit) {
                stats.hit_ray_count++;
            } else {
                stats.no_return_ray_count++;
            }

            const std::size_t occupied_index =
                hit ? traversal.cells.size() - 1U : traversal.cells.size();
            for (std::size_t i = 0; i < traversal.cells.size(); ++i) {
                const bool occupied = hit && i == occupied_index;
                auto inserted = candidate.emplace(traversal.cells[i], occupied);
                if (!inserted.second) {
                    stats.duplicate_cell_suppression_count++;
                    if (occupied && !inserted.first->second) {
                        inserted.first->second = true;
                        stats.occupied_over_free_resolution_count++;
                    }
                }
            }
        }

        if (stats.valid_ray_count == 0) {
            stats.fault = SparseOccupancyGridFault::NoUsableRay;
            stats.message = "sparse_grid_no_usable_ray";
            return stats;
        }

        std::size_t new_cell_count = 0;
        for (const auto &entry : candidate) {
            if (cells_.find(entry.first) == cells_.end()) {
                new_cell_count++;
            }
        }
        if (cells_.size() + new_cell_count > config_.maximum_active_cells) {
            stats.fault = SparseOccupancyGridFault::MapCapacityReached;
            stats.message = "sparse_grid_capacity_reached";
            return stats;
        }

        for (const auto &entry : candidate) {
            auto &cell = cells_[entry.first];
            const std::int16_t delta =
                entry.second ? config_.occupied_delta : config_.free_delta;
            const int next = static_cast<int>(cell.evidence) +
                             static_cast<int>(delta);
            if (next < static_cast<int>(config_.minimum_evidence)) {
                cell.evidence = config_.minimum_evidence;
            } else if (next > static_cast<int>(config_.maximum_evidence)) {
                cell.evidence = config_.maximum_evidence;
            } else {
                cell.evidence = static_cast<std::int16_t>(next);
            }
            if (entry.second) {
                stats.occupied_cell_update_count++;
            } else {
                stats.free_cell_update_count++;
            }
        }

        stats.accepted = true;
        stats.fault = SparseOccupancyGridFault::None;
        stats.message = "sparse_grid_update_accepted";
        return stats;
    }

    SparseOccupancyGridSnapshot snapshot() const {
        SparseOccupancyGridSnapshot out;
        out.resolution_m = config_.resolution_m;
        bool first = true;
        for (const auto &entry : cells_) {
            SparseOccupancyCell cell;
            cell.key = entry.first;
            cell.evidence = entry.second.evidence;
            cell.free = cell.evidence <= config_.free_threshold;
            cell.occupied = cell.evidence >= config_.occupied_threshold;
            if (cell.free) {
                out.free_cell_count++;
            } else if (cell.occupied) {
                out.occupied_cell_count++;
            } else {
                out.uncertain_cell_count++;
            }
            if (first) {
                out.min_x = out.max_x = cell.key.x;
                out.min_y = out.max_y = cell.key.y;
                first = false;
            } else {
                out.min_x = std::min(out.min_x, cell.key.x);
                out.max_x = std::max(out.max_x, cell.key.x);
                out.min_y = std::min(out.min_y, cell.key.y);
                out.max_y = std::max(out.max_y, cell.key.y);
            }
            out.cells.push_back(cell);
        }
        return out;
    }

    std::size_t active_cell_count() const {
        return cells_.size();
    }

    bool contains(const SparseGridCellKey &key) const {
        return cells_.find(key) != cells_.end();
    }

    std::int16_t evidence(const SparseGridCellKey &key) const {
        const auto it = cells_.find(key);
        return it == cells_.end() ? 0 : it->second.evidence;
    }

private:
    SparseOccupancyGridConfig config_;
    std::map<SparseGridCellKey, SparseOccupancyCellState> cells_;
};

} // namespace robot_slamd
