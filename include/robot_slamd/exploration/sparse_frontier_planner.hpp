#pragma once

#include "robot_slamd/exploration/bounded_astar.hpp"
#include "robot_slamd/exploration/frontier_failure_memory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace robot_slamd {

struct SparseFrontierPlannerConfig {
    std::size_t minimum_cluster_cells = 2;
    std::size_t maximum_frontier_cells = 50000;
    std::size_t maximum_clusters = 2048;
    double information_gain_weight = 0.02;
    double minimum_goal_clearance_m = 0.10;
    double minimum_goal_distance_m = 0.20;
    double obstacle_clearance_penalty_weight = 0.03;
    double repeated_failure_penalty = 0.20;
    int maximum_goal_search_radius_cells = 6;
    int completed_goal_exclusion_radius_cells = 3;
    BoundedAStarConfig astar;
};

struct SparseFrontierCluster {
    std::uint64_t id = 0;
    std::vector<SparseGridCellKey> cells;
};

struct SparseFrontierPlanResult {
    bool ok = false;
    bool no_reachable_frontier = false;
    std::string reason;
    std::size_t detected_frontier_cells = 0;
    std::size_t cluster_count = 0;
    std::size_t reachable_cluster_count = 0;
    std::size_t unreachable_cluster_count = 0;
    std::size_t cooled_cluster_count = 0;
    std::size_t permanently_failed_cluster_count = 0;
    std::size_t expanded_astar_nodes = 0;
    std::uint64_t selected_frontier_id = 0;
    std::size_t selected_cluster_size = 0;
    SparseGridCellKey selected_goal_cell;
    double selected_goal_clearance_m = 0.0;
    double selected_score = 0.0;
    BoundedAStarResult path;
};

class SparseFrontierPlanner {
public:
    explicit SparseFrontierPlanner(SparseFrontierPlannerConfig config = {})
        : config_(config), astar_(config.astar) {}

    SparseFrontierPlanResult plan(
        const NavigationGridView &grid,
        const SparseGridCellKey &start,
        const std::set<std::uint64_t> &excluded_frontiers = {},
        const FrontierFailureMemory *failure_memory = nullptr,
        const std::vector<SparseGridCellKey> &completed_goals = {}) const {
        SparseFrontierPlanResult result;
        if (!grid.valid() || !grid.traversable(start) ||
            config_.minimum_cluster_cells == 0 ||
            config_.maximum_frontier_cells == 0 ||
            config_.maximum_clusters == 0 ||
            !std::isfinite(config_.information_gain_weight) ||
            config_.information_gain_weight < 0.0 ||
            !std::isfinite(config_.minimum_goal_clearance_m) ||
            config_.minimum_goal_clearance_m < 0.0 ||
            !std::isfinite(config_.minimum_goal_distance_m) ||
            config_.minimum_goal_distance_m < 0.0 ||
            !std::isfinite(config_.obstacle_clearance_penalty_weight) ||
            config_.obstacle_clearance_penalty_weight < 0.0 ||
            !std::isfinite(config_.repeated_failure_penalty) ||
            config_.repeated_failure_penalty < 0.0 ||
            config_.maximum_goal_search_radius_cells <= 0 ||
            config_.completed_goal_exclusion_radius_cells < 0) {
            result.reason = "frontier_planner_invalid_input";
            return result;
        }
        const auto clusters = detect_clusters(grid, result);
        if (!result.reason.empty()) return result;
        result.cluster_count = clusters.size();
        bool have_best = false;
        double best_score = std::numeric_limits<double>::infinity();
        for (const auto &cluster : clusters) {
            if (excluded_frontiers.count(cluster.id) != 0) continue;
            BoundedAStarResult cluster_path;
            SparseGridCellKey cluster_goal;
            double cluster_clearance = 0.0;
            double cluster_score = std::numeric_limits<double>::infinity();
            bool cluster_cooled = false;
            bool cluster_permanent = false;
            for (const auto &candidate : safe_goal_candidates(cluster, grid, start)) {
                if (near_any(candidate, completed_goals,
                             config_.completed_goal_exclusion_radius_cells)) {
                    continue;
                }
                const auto failure = failure_memory
                    ? failure_memory->assess(candidate) : FrontierFailureAssessment{};
                cluster_cooled = cluster_cooled || failure.cooled;
                cluster_permanent = cluster_permanent || failure.permanently_failed;
                if (failure.cooled || failure.permanently_failed) continue;
                auto path = astar_.plan(grid, start, candidate);
                result.expanded_astar_nodes += path.expanded_nodes;
                if (!path.ok) {
                    if (path.reason == "astar_expansion_budget_exceeded") {
                        result.reason = path.reason;
                        return result;
                    }
                    continue;
                }
                if (path.path_length_m + 1e-12 <
                    config_.minimum_goal_distance_m) {
                    continue;
                }
                const double clearance = grid.clearance_to_blocked_m(
                    candidate, config_.maximum_goal_search_radius_cells);
                const double clearance_penalty =
                    config_.obstacle_clearance_penalty_weight /
                    std::max(clearance, grid.resolution_m());
                const double score = path.path_length_m + clearance_penalty +
                    config_.repeated_failure_penalty * failure.failure_count -
                    config_.information_gain_weight * cluster.cells.size();
                if (!cluster_path.ok || score < cluster_score - 1e-12 ||
                    (std::fabs(score - cluster_score) <= 1e-12 &&
                     candidate < cluster_goal)) {
                    cluster_path = std::move(path);
                    cluster_goal = candidate;
                    cluster_clearance = clearance;
                    cluster_score = score;
                }
                break;
            }
            if (!cluster_path.ok) {
                if (cluster_permanent) result.permanently_failed_cluster_count++;
                else if (cluster_cooled) result.cooled_cluster_count++;
                result.unreachable_cluster_count++;
                continue;
            }
            result.reachable_cluster_count++;
            if (!have_best || cluster_score < best_score - 1e-12 ||
                (std::fabs(cluster_score - best_score) <= 1e-12 &&
                 cluster.id < result.selected_frontier_id)) {
                have_best = true;
                best_score = cluster_score;
                result.selected_frontier_id = cluster.id;
                result.selected_cluster_size = cluster.cells.size();
                result.selected_goal_cell = cluster_goal;
                result.selected_goal_clearance_m = cluster_clearance;
                result.path = std::move(cluster_path);
            }
        }
        if (!have_best) {
            result.ok = true;
            result.no_reachable_frontier = true;
            result.reason = "no_reachable_frontier";
            return result;
        }
        result.ok = true;
        result.selected_score = best_score;
        result.reason = "reachable_frontier_selected";
        return result;
    }

private:
    static bool near_any(const SparseGridCellKey &candidate,
                         const std::vector<SparseGridCellKey> &goals,
                         int radius) {
        return std::any_of(goals.begin(), goals.end(),
            [&candidate, radius](const SparseGridCellKey &goal) {
                return std::abs(candidate.x - goal.x) <= radius &&
                       std::abs(candidate.y - goal.y) <= radius;
            });
    }

    std::vector<SparseFrontierCluster> detect_clusters(
        const NavigationGridView &grid,
        SparseFrontierPlanResult &result) const {
        std::set<SparseGridCellKey> frontier;
        static constexpr int kFour[4][2] = {{1,0},{0,1},{-1,0},{0,-1}};
        for (std::size_t y = 0; y < grid.height(); ++y) {
            for (std::size_t x = 0; x < grid.width(); ++x) {
                const auto key = grid.global_key(x, y);
                if (!grid.traversable(key)) continue;
                bool adjacent_unknown = false;
                for (const auto &offset : kFour) {
                    if (grid.cell({key.x + offset[0], key.y + offset[1]}) ==
                        NavigationCellClass::Unknown) {
                        adjacent_unknown = true;
                        break;
                    }
                }
                if (adjacent_unknown) {
                    frontier.insert(key);
                    if (frontier.size() > config_.maximum_frontier_cells) {
                        result.reason = "frontier_cell_budget_exceeded";
                        return {};
                    }
                }
            }
        }
        result.detected_frontier_cells = frontier.size();
        std::vector<SparseFrontierCluster> clusters;
        static constexpr int kEight[8][2] = {
            {1,0},{0,1},{-1,0},{0,-1},{1,1},{-1,1},{-1,-1},{1,-1}};
        while (!frontier.empty()) {
            SparseFrontierCluster cluster;
            std::queue<SparseGridCellKey> pending;
            pending.push(*frontier.begin());
            frontier.erase(frontier.begin());
            while (!pending.empty()) {
                const auto key = pending.front();
                pending.pop();
                cluster.cells.push_back(key);
                for (const auto &offset : kEight) {
                    const SparseGridCellKey next{key.x + offset[0],
                                                 key.y + offset[1]};
                    const auto found = frontier.find(next);
                    if (found != frontier.end()) {
                        pending.push(*found);
                        frontier.erase(found);
                    }
                }
            }
            std::sort(cluster.cells.begin(), cluster.cells.end());
            if (cluster.cells.size() < config_.minimum_cluster_cells) continue;
            if (clusters.size() >= config_.maximum_clusters) {
                result.reason = "frontier_cluster_budget_exceeded";
                return {};
            }
            cluster.id = stable_cluster_id(cluster.cells);
            clusters.push_back(std::move(cluster));
        }
        return clusters;
    }

    static std::uint64_t stable_cluster_id(
        const std::vector<SparseGridCellKey> &cells) {
        std::uint64_t hash = 1469598103934665603ULL;
        for (const auto &cell : cells) {
            const auto mix = [&hash](std::uint32_t value) {
                for (int shift = 0; shift < 32; shift += 8) {
                    hash ^= static_cast<std::uint8_t>(value >> shift);
                    hash *= 1099511628211ULL;
                }
            };
            mix(static_cast<std::uint32_t>(cell.x));
            mix(static_cast<std::uint32_t>(cell.y));
        }
        return hash == 0 ? 1 : hash;
    }

    std::vector<SparseGridCellKey> safe_goal_candidates(
        const SparseFrontierCluster &cluster,
        const NavigationGridView &grid,
        const SparseGridCellKey &start) const {
        std::set<SparseGridCellKey> unique;
        std::set<SparseGridCellKey> visited(cluster.cells.begin(),
                                            cluster.cells.end());
        std::queue<std::pair<SparseGridCellKey, int>> pending;
        for (const auto &frontier : cluster.cells) {
            pending.push({frontier, 0});
        }
        static constexpr int kFour[4][2] = {{1,0},{0,1},{-1,0},{0,-1}};
        while (!pending.empty()) {
            const auto current = pending.front();
            pending.pop();
            if (grid.clearance_to_non_free_m(
                    current.first, config_.maximum_goal_search_radius_cells) +
                    1e-12 >= config_.minimum_goal_clearance_m) {
                unique.insert(current.first);
            }
            if (current.second >= config_.maximum_goal_search_radius_cells) {
                continue;
            }
            for (const auto &offset : kFour) {
                const SparseGridCellKey next{current.first.x + offset[0],
                                             current.first.y + offset[1]};
                if (grid.traversable(next) && visited.insert(next).second) {
                    pending.push({next, current.second + 1});
                }
            }
        }
        std::vector<SparseGridCellKey> candidates(unique.begin(), unique.end());
        std::sort(candidates.begin(), candidates.end(),
            [&grid, &start, this](const SparseGridCellKey &a,
                                 const SparseGridCellKey &b) {
                const double ca = grid.clearance_to_non_free_m(
                    a, config_.maximum_goal_search_radius_cells);
                const double cb = grid.clearance_to_non_free_m(
                    b, config_.maximum_goal_search_radius_cells);
                if (std::fabs(ca - cb) > 1e-12) return ca > cb;
                const auto da = std::abs(a.x - start.x) + std::abs(a.y - start.y);
                const auto db = std::abs(b.x - start.x) + std::abs(b.y - start.y);
                if (da != db) return da < db;
                return a < b;
            });
        return candidates;
    }

    SparseFrontierPlannerConfig config_;
    BoundedAStarPlanner astar_;
};

} // namespace robot_slamd
