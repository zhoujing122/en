#pragma once

#include "robot_slamd/exploration/bounded_astar.hpp"

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
    std::size_t expanded_astar_nodes = 0;
    std::uint64_t selected_frontier_id = 0;
    std::size_t selected_cluster_size = 0;
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
        const std::set<std::uint64_t> &excluded_frontiers = {}) const {
        SparseFrontierPlanResult result;
        if (!grid.valid() || !grid.traversable(start) ||
            config_.minimum_cluster_cells == 0 ||
            config_.maximum_frontier_cells == 0 ||
            config_.maximum_clusters == 0 ||
            !std::isfinite(config_.information_gain_weight) ||
            config_.information_gain_weight < 0.0) {
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
            for (const auto &candidate : representative_order(cluster, start)) {
                cluster_path = astar_.plan(grid, start, candidate);
                result.expanded_astar_nodes += cluster_path.expanded_nodes;
                if (cluster_path.ok) break;
                if (cluster_path.reason == "astar_expansion_budget_exceeded") {
                    result.reason = cluster_path.reason;
                    return result;
                }
            }
            if (!cluster_path.ok) {
                result.unreachable_cluster_count++;
                continue;
            }
            result.reachable_cluster_count++;
            const double score = cluster_path.path_length_m -
                config_.information_gain_weight *
                    static_cast<double>(cluster.cells.size());
            if (!have_best || score < best_score - 1e-12 ||
                (std::fabs(score - best_score) <= 1e-12 &&
                 cluster.id < result.selected_frontier_id)) {
                have_best = true;
                best_score = score;
                result.selected_frontier_id = cluster.id;
                result.selected_cluster_size = cluster.cells.size();
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

    static std::vector<SparseGridCellKey> representative_order(
        const SparseFrontierCluster &cluster,
        const SparseGridCellKey &start) {
        auto candidates = cluster.cells;
        std::sort(candidates.begin(), candidates.end(),
            [&start](const SparseGridCellKey &a, const SparseGridCellKey &b) {
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
