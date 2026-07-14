#pragma once

#include "robot_slamd/exploration/navigation_grid_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <string>
#include <vector>

namespace robot_slamd {

struct BoundedAStarConfig {
    bool allow_diagonal = true;
    std::size_t maximum_expanded_nodes = 50000;
};

struct BoundedAStarResult {
    bool ok = false;
    std::string reason;
    std::vector<SparseGridCellKey> cells;
    std::vector<RobotPose2D> waypoints;
    double path_length_m = 0.0;
    std::size_t expanded_nodes = 0;
};

class BoundedAStarPlanner {
public:
    explicit BoundedAStarPlanner(BoundedAStarConfig config = {})
        : config_(config) {}

    BoundedAStarResult plan(const NavigationGridView &grid,
                            const SparseGridCellKey &start,
                            const SparseGridCellKey &goal) const {
        BoundedAStarResult result;
        if (!grid.valid() || config_.maximum_expanded_nodes == 0 ||
            !grid.traversable(start) || !grid.traversable(goal)) {
            result.reason = "astar_invalid_or_blocked_endpoint";
            return result;
        }
        const std::size_t count = grid.cell_count();
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> g(count, inf);
        std::vector<std::int64_t> parent(count, -1);
        std::vector<bool> closed(count, false);
        const auto origin = grid.global_key(0, 0);
        const auto to_index = [&grid, &origin](const SparseGridCellKey &key) {
            return static_cast<std::size_t>(key.y - origin.y) * grid.width() +
                   static_cast<std::size_t>(key.x - origin.x);
        };
        const auto from_index = [&grid](std::size_t index) {
            return grid.global_key(index % grid.width(), index / grid.width());
        };
        struct OpenNode { double f; double g; std::size_t index; };
        struct Worse {
            bool operator()(const OpenNode &a, const OpenNode &b) const {
                if (a.f != b.f) return a.f > b.f;
                if (a.g != b.g) return a.g > b.g;
                return a.index > b.index;
            }
        };
        std::priority_queue<OpenNode, std::vector<OpenNode>, Worse> open;
        const auto heuristic = [&goal](const SparseGridCellKey &key) {
            const double dx = std::fabs(static_cast<double>(goal.x - key.x));
            const double dy = std::fabs(static_cast<double>(goal.y - key.y));
            const double diagonal = std::min(dx, dy);
            return diagonal * std::sqrt(2.0) + std::max(dx, dy) - diagonal;
        };
        const auto start_index = to_index(start);
        const auto goal_index = to_index(goal);
        g[start_index] = 0.0;
        open.push({heuristic(start), 0.0, start_index});
        static constexpr int kNeighbors[8][2] = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1},
            {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
        bool reached = false;
        while (!open.empty()) {
            const auto current = open.top();
            open.pop();
            if (closed[current.index]) continue;
            closed[current.index] = true;
            result.expanded_nodes++;
            if (result.expanded_nodes > config_.maximum_expanded_nodes) {
                result.reason = "astar_expansion_budget_exceeded";
                return result;
            }
            if (current.index == goal_index) {
                reached = true;
                break;
            }
            const auto key = from_index(current.index);
            const int neighbor_count = config_.allow_diagonal ? 8 : 4;
            for (int i = 0; i < neighbor_count; ++i) {
                const int dx = kNeighbors[i][0];
                const int dy = kNeighbors[i][1];
                const SparseGridCellKey next{key.x + dx, key.y + dy};
                if (!grid.contains(next) || !grid.traversable(next)) continue;
                if (dx != 0 && dy != 0 &&
                    (!grid.traversable({key.x + dx, key.y}) ||
                     !grid.traversable({key.x, key.y + dy}))) {
                    continue;
                }
                const auto next_index = to_index(next);
                if (closed[next_index]) continue;
                const double step_cost = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
                const double candidate = g[current.index] + step_cost;
                if (candidate + 1e-12 < g[next_index]) {
                    g[next_index] = candidate;
                    parent[next_index] = static_cast<std::int64_t>(current.index);
                    open.push({candidate + heuristic(next), candidate, next_index});
                }
            }
        }
        if (!reached) {
            result.reason = "astar_no_path";
            return result;
        }
        for (std::int64_t at = static_cast<std::int64_t>(goal_index); at >= 0;
             at = parent[static_cast<std::size_t>(at)]) {
            result.cells.push_back(from_index(static_cast<std::size_t>(at)));
            if (static_cast<std::size_t>(at) == start_index) break;
        }
        std::reverse(result.cells.begin(), result.cells.end());
        simplify_collinear(result.cells);
        result.waypoints.reserve(result.cells.size());
        for (const auto &cell : result.cells) {
            result.waypoints.push_back(grid.cell_center(cell));
        }
        result.path_length_m = g[goal_index] * grid.resolution_m();
        result.ok = true;
        result.reason = "astar_path_ready";
        return result;
    }

private:
    static void simplify_collinear(std::vector<SparseGridCellKey> &path) {
        if (path.size() < 3) return;
        std::vector<SparseGridCellKey> simplified;
        simplified.reserve(path.size());
        simplified.push_back(path.front());
        for (std::size_t i = 1; i + 1 < path.size(); ++i) {
            const int dx1 = path[i].x - path[i - 1].x;
            const int dy1 = path[i].y - path[i - 1].y;
            const int dx2 = path[i + 1].x - path[i].x;
            const int dy2 = path[i + 1].y - path[i].y;
            if (dx1 != dx2 || dy1 != dy2) simplified.push_back(path[i]);
        }
        simplified.push_back(path.back());
        path.swap(simplified);
    }

    BoundedAStarConfig config_;
};

} // namespace robot_slamd
