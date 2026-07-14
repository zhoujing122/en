#include "robot_slamd/exploration/frontier_failure_memory.hpp"
#include "robot_slamd/exploration/sparse_frontier_planner.hpp"
#include "sparse_navigation_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace sparse_navigation_test;

    std::vector<SparseOccupancyCell> cells;
    for (int y = 2; y <= 10; ++y) {
        for (int x = 2; x <= 12; ++x) {
            if (x != 8 || y != 6) cells.push_back(free_cell(x, y));
        }
    }
    cells.push_back(occupied_cell(8, 6));
    NavigationGridViewConfig view_config;
    view_config.min_x_m = 0.0;
    view_config.max_x_m = 15.0;
    view_config.min_y_m = 0.0;
    view_config.max_y_m = 13.0;
    view_config.robot_radius_m = 0.0;
    view_config.safety_margin_m = 0.0;
    NavigationGridView grid;
    expect(grid.build(snapshot_from_cells(cells), view_config).ok,
           "quality planning grid builds");

    SparseFrontierPlannerConfig config;
    config.minimum_cluster_cells = 2;
    config.minimum_goal_clearance_m = 2.0;
    config.maximum_goal_search_radius_cells = 4;
    config.astar.maximum_expanded_nodes = 2000;
    SparseFrontierPlanner planner(config);
    const auto first = planner.plan(grid, {4, 6});
    expect(first.ok && !first.no_reachable_frontier && first.path.ok,
           "safe reachable frontier goal selected");
    expect(grid.traversable(first.selected_goal_cell) &&
               grid.clearance_to_non_free_m(first.selected_goal_cell, 4) >= 2.0,
           "goal is inside known free space with required clearance");
    expect(first.path.cells.back() == first.selected_goal_cell,
           "A-star validates selected goal");

    FrontierFailureMemoryConfig memory_config;
    memory_config.cooldown_planning_cycles = 2;
    memory_config.maximum_consecutive_failures = 3;
    memory_config.revision_change_for_retry = 10;
    memory_config.spatial_match_radius_cells = 0;
    FrontierFailureMemory memory(memory_config);
    expect(memory.record(first.selected_goal_cell, "obstacle_blocked", 5),
           "failed goal recorded");
    expect(memory.assess(first.selected_goal_cell).cooled,
           "failed goal enters cooldown");
    memory.begin_planning_cycle(6);
    expect(memory.assess(first.selected_goal_cell).cooled,
           "one cycle does not immediately retry");
    memory.begin_planning_cycle(15);
    expect(!memory.assess(first.selected_goal_cell).cooled,
           "significant map change allows reevaluation");
    memory.record(first.selected_goal_cell, "no_progress", 15);
    memory.record(first.selected_goal_cell, "path_blocked", 16);
    expect(memory.assess(first.selected_goal_cell).permanently_failed,
           "bounded repeated failure retires only matching goal");
    expect(!memory.assess({first.selected_goal_cell.x + 1,
                           first.selected_goal_cell.y}).permanently_failed,
           "neighboring frontier is not broadly blacklisted");
    return 0;
}
