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

class SparseOccupancyGridPreparedBatch;
struct SparseOccupancyGridBatchPrepareResult;

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


    SparseOccupancyGridSnapshotCaptureResult capture_snapshot(
        std::uint64_t revision,
        std::size_t maximum_snapshot_cells) const {
        SparseOccupancyGridSnapshotCaptureResult result;
        if (!valid()) {
            result.fault = SparseOccupancyGridSnapshotFault::InvalidGrid;
            result.message = "reference_snapshot_grid_invalid";
            return result;
        }
        if (maximum_snapshot_cells == 0) {
            result.fault = SparseOccupancyGridSnapshotFault::InvalidLimit;
            result.message = "reference_snapshot_limit_invalid";
            return result;
        }
        if (cells_.size() > maximum_snapshot_cells) {
            result.fault = SparseOccupancyGridSnapshotFault::CapacityExceeded;
            result.message = "reference_snapshot_capacity_exceeded";
            return result;
        }

        SparseOccupancyGridSnapshot out;
        out.valid_ = true;
        out.revision_ = revision;
        out.resolution_m_ = config_.resolution_m;
        out.free_threshold_ = config_.free_threshold;
        out.occupied_threshold_ = config_.occupied_threshold;
        out.cells_.reserve(cells_.size());
        bool first = true;
        for (const auto &entry : cells_) {
            SparseOccupancyCell cell;
            cell.key = entry.first;
            cell.evidence = entry.second.evidence;
            if (cell.evidence <= config_.free_threshold) {
                out.free_cell_count_++;
            } else if (cell.evidence >= config_.occupied_threshold) {
                out.occupied_cell_count_++;
            } else {
                out.uncertain_cell_count_++;
            }
            if (first) {
                out.min_x_ = out.max_x_ = cell.key.x;
                out.min_y_ = out.max_y_ = cell.key.y;
                out.has_bounds_ = true;
                first = false;
            } else {
                out.min_x_ = std::min(out.min_x_, cell.key.x);
                out.max_x_ = std::max(out.max_x_, cell.key.x);
                out.min_y_ = std::min(out.min_y_, cell.key.y);
                out.max_y_ = std::max(out.max_y_, cell.key.y);
            }
            out.cells_.push_back(cell);
        }
        result.ok = true;
        result.snapshot = std::move(out);
        result.message = "reference_snapshot_captured";
        return result;
    }

    SparseOccupancyGridSnapshot snapshot() const {
        const auto captured = capture_snapshot(0, config_.maximum_active_cells);
        return captured.snapshot;
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

    SparseOccupancyGridBatchPrepareResult prepare_observation_batch(
        const std::vector<std::vector<SparseTofRayObservation>> &batches,
        std::size_t maximum_changed_cells) const;

    bool commit_prepared_batch(
        SparseOccupancyGridPreparedBatch &&prepared) noexcept;

    bool replace_cells_transactional(
        const std::vector<SparseOccupancyCell> &cells) {
        if (!valid() || cells.size() > config_.maximum_active_cells) {
            return false;
        }
        std::map<SparseGridCellKey, SparseOccupancyCellState> staged;
        for (const auto &cell : cells) {
            if (cell.evidence < config_.minimum_evidence ||
                cell.evidence > config_.maximum_evidence ||
                !staged.emplace(cell.key,
                                SparseOccupancyCellState{cell.evidence})
                     .second) {
                return false;
            }
        }
        cells_.swap(staged);
        return true;
    }

private:
    SparseOccupancyGridConfig config_;
    std::map<SparseGridCellKey, SparseOccupancyCellState> cells_;
};

struct SparseOccupancyGridBatchStats {
    std::size_t batch_count = 0;
    std::size_t valid_ray_count = 0;
    std::size_t hit_ray_count = 0;
    std::size_t no_return_ray_count = 0;
    std::size_t changed_cell_count = 0;
    std::size_t free_cell_update_count = 0;
    std::size_t occupied_cell_update_count = 0;
    std::size_t final_cell_count = 0;
    std::size_t final_free_cell_count = 0;
    std::size_t final_occupied_cell_count = 0;
    std::size_t final_uncertain_cell_count = 0;
};

class SparseOccupancyGridPreparedBatch {
public:
    bool ready() const { return ready_; }
    const SparseOccupancyGridBatchStats &stats() const { return stats_; }

private:
    friend class SparseOccupancyGrid;
    bool ready_ = false;
    SparseOccupancyGrid staged_grid_;
    std::map<SparseGridCellKey, SparseOccupancyCellState> baseline_cells_;
    SparseOccupancyGridBatchStats stats_;
};

struct SparseOccupancyGridBatchPrepareResult {
    bool ok = false;
    SparseOccupancyGridPreparedBatch prepared;
    SparseOccupancyGridFault fault = SparseOccupancyGridFault::None;
    std::string message;
};

inline SparseOccupancyGridBatchPrepareResult
SparseOccupancyGrid::prepare_observation_batch(
    const std::vector<std::vector<SparseTofRayObservation>> &batches,
    std::size_t maximum_changed_cells) const {
    SparseOccupancyGridBatchPrepareResult result;
    if (!valid() || batches.empty() || maximum_changed_cells == 0) {
        result.fault = SparseOccupancyGridFault::InvalidConfig;
        result.message = "sparse_grid_batch_invalid_request";
        return result;
    }

    result.prepared.staged_grid_ = *this;
    result.prepared.baseline_cells_ = cells_;
    for (const auto &batch : batches) {
        const auto stats = result.prepared.staged_grid_.apply_observations(batch);
        if (!stats.accepted) {
            result.fault = stats.fault;
            result.message = stats.message;
            return result;
        }
        result.prepared.stats_.batch_count++;
        result.prepared.stats_.valid_ray_count +=
            static_cast<std::size_t>(stats.valid_ray_count);
        result.prepared.stats_.hit_ray_count +=
            static_cast<std::size_t>(stats.hit_ray_count);
        result.prepared.stats_.no_return_ray_count +=
            static_cast<std::size_t>(stats.no_return_ray_count);
        result.prepared.stats_.free_cell_update_count +=
            static_cast<std::size_t>(stats.free_cell_update_count);
        result.prepared.stats_.occupied_cell_update_count +=
            static_cast<std::size_t>(stats.occupied_cell_update_count);
    }

    for (const auto &entry : result.prepared.staged_grid_.cells_) {
        const auto before = cells_.find(entry.first);
        if (before == cells_.end() ||
            before->second.evidence != entry.second.evidence) {
            result.prepared.stats_.changed_cell_count++;
        }
        if (entry.second.evidence <= config_.free_threshold) {
            result.prepared.stats_.final_free_cell_count++;
        } else if (entry.second.evidence >= config_.occupied_threshold) {
            result.prepared.stats_.final_occupied_cell_count++;
        } else {
            result.prepared.stats_.final_uncertain_cell_count++;
        }
    }
    result.prepared.stats_.final_cell_count =
        result.prepared.staged_grid_.cells_.size();
    if (result.prepared.stats_.changed_cell_count > maximum_changed_cells) {
        result.fault = SparseOccupancyGridFault::MapCapacityReached;
        result.message = "sparse_grid_batch_changed_cell_limit";
        return result;
    }
    result.prepared.ready_ = true;
    result.ok = true;
    result.message = "sparse_grid_batch_prepared";
    return result;
}

inline bool SparseOccupancyGrid::commit_prepared_batch(
    SparseOccupancyGridPreparedBatch &&prepared) noexcept {
    if (!prepared.ready_ || cells_.size() != prepared.baseline_cells_.size()) {
        return false;
    }
    auto current = cells_.begin();
    auto baseline = prepared.baseline_cells_.begin();
    for (; current != cells_.end(); ++current, ++baseline) {
        if (!(current->first == baseline->first) ||
            current->second.evidence != baseline->second.evidence) {
            return false;
        }
    }
    cells_.swap(prepared.staged_grid_.cells_);
    prepared.ready_ = false;
    return true;
}

} // namespace robot_slamd
