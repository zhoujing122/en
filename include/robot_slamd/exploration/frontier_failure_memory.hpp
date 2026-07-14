#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_grid_types.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_slamd {

struct FrontierFailureMemoryConfig {
    std::size_t maximum_entries = 64;
    std::size_t cooldown_planning_cycles = 2;
    std::size_t maximum_consecutive_failures = 3;
    std::uint64_t revision_change_for_retry = 20;
    int spatial_match_radius_cells = 2;
};

struct FrontierFailureRecord {
    SparseGridCellKey goal_cell;
    std::string reason;
    std::size_t failure_count = 0;
    std::uint64_t map_revision = 0;
    std::size_t cooldown_remaining = 0;
    bool permanently_failed = false;
};

struct FrontierFailureAssessment {
    bool cooled = false;
    bool permanently_failed = false;
    std::size_t failure_count = 0;
};

class FrontierFailureMemory {
public:
    explicit FrontierFailureMemory(FrontierFailureMemoryConfig config = {})
        : config_(config) {}

    bool valid() const {
        return config_.maximum_entries > 0 &&
               config_.maximum_consecutive_failures > 0 &&
               config_.spatial_match_radius_cells >= 0;
    }

    void begin_planning_cycle(std::uint64_t revision) {
        for (auto &entry : entries_) {
            if (!entry.permanently_failed &&
                revision >= entry.map_revision + config_.revision_change_for_retry) {
                entry.cooldown_remaining = 0;
            } else if (entry.cooldown_remaining > 0) {
                entry.cooldown_remaining--;
            }
        }
    }

    bool record(const SparseGridCellKey &goal, const std::string &reason,
                std::uint64_t revision) {
        if (!valid() || reason.empty()) return false;
        auto *entry = find_mutable(goal);
        if (!entry) {
            if (entries_.size() >= config_.maximum_entries) {
                entries_.erase(entries_.begin());
            }
            entries_.push_back({goal, reason, 0, revision, 0, false});
            entry = &entries_.back();
        }
        entry->goal_cell = goal;
        entry->reason = reason;
        entry->failure_count++;
        entry->map_revision = revision;
        entry->cooldown_remaining = config_.cooldown_planning_cycles;
        entry->permanently_failed =
            entry->failure_count >= config_.maximum_consecutive_failures;
        return true;
    }

    FrontierFailureAssessment assess(const SparseGridCellKey &goal) const {
        const auto *entry = find(goal);
        if (!entry) return {};
        return {entry->cooldown_remaining > 0, entry->permanently_failed,
                entry->failure_count};
    }

    std::size_t size() const { return entries_.size(); }
    const std::vector<FrontierFailureRecord> &entries() const { return entries_; }

private:
    bool matches(const SparseGridCellKey &a, const SparseGridCellKey &b) const {
        return std::abs(a.x - b.x) <= config_.spatial_match_radius_cells &&
               std::abs(a.y - b.y) <= config_.spatial_match_radius_cells;
    }

    const FrontierFailureRecord *find(const SparseGridCellKey &goal) const {
        const auto it = std::find_if(entries_.begin(), entries_.end(),
            [this, &goal](const FrontierFailureRecord &entry) {
                return matches(entry.goal_cell, goal);
            });
        return it == entries_.end() ? nullptr : &*it;
    }

    FrontierFailureRecord *find_mutable(const SparseGridCellKey &goal) {
        const auto it = std::find_if(entries_.begin(), entries_.end(),
            [this, &goal](const FrontierFailureRecord &entry) {
                return matches(entry.goal_cell, goal);
            });
        return it == entries_.end() ? nullptr : &*it;
    }

    FrontierFailureMemoryConfig config_;
    std::vector<FrontierFailureRecord> entries_;
};

} // namespace robot_slamd
