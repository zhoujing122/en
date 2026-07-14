#include "sparse_navigation_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace sparse_navigation_test;
    std::vector<SparseOccupancyCell> cells;
    for (int y = 0; y <= 6; ++y) {
        for (int x = 0; x <= 8; ++x) {
            if (x == 4 && y >= 1 && y <= 4) continue;
            cells.push_back(free_cell(x, y));
        }
    }
    for (int y = 1; y <= 4; ++y) cells.push_back(occupied_cell(4, y));
    cells.push_back(free_cell(11, 1));
    cells.push_back(free_cell(11, 2));
    cells.push_back(free_cell(12, 1));
    cells.push_back(free_cell(12, 2));

    NavigationGridViewConfig view_config;
    view_config.min_x_m = 0.0;
    view_config.max_x_m = 14.0;
    view_config.min_y_m = 0.0;
    view_config.max_y_m = 9.0;
    view_config.robot_radius_m = 0.0;
    view_config.safety_margin_m = 0.0;
    NavigationGridView grid;
    expect(grid.build(snapshot_from_cells(cells), view_config).ok,
           "frontier planning view builds");

    BoundedAStarPlanner astar({true, 500});
    const auto path = astar.plan(grid, {1, 2}, {7, 2});
    expect(path.ok && path.expanded_nodes > 0, "astar finds bounded path");
    bool crossed_wall = false;
    for (const auto &cell : path.cells) {
        crossed_wall = crossed_wall || (cell.x == 4 && cell.y <= 4);
    }
    expect(!crossed_wall, "astar routes around occupied wall");

    SparseFrontierPlannerConfig planner_config;
    planner_config.minimum_cluster_cells = 2;
    planner_config.maximum_frontier_cells = 200;
    planner_config.astar.maximum_expanded_nodes = 500;
    SparseFrontierPlanner planner(planner_config);
    const auto selected = planner.plan(grid, {1, 2});
    expect(selected.ok && !selected.no_reachable_frontier,
           "reachable frontier selected");
    expect(selected.reachable_cluster_count >= 1 &&
           selected.unreachable_cluster_count >= 1,
           "unreachable frontier skipped");
    expect(selected.path.ok && selected.path.waypoints.size() >= 2,
           "selected frontier has world path");

    BoundedAStarPlanner budgeted({true, 1});
    expect(budgeted.plan(grid, {1, 2}, {7, 2}).reason ==
               "astar_expansion_budget_exceeded",
           "astar budget fails explicitly");
    return 0;
}
