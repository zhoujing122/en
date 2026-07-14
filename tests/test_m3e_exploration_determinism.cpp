#include "m3e_exploration_test_helpers.hpp"

int main() {
    const auto first = m3e_test::run("/tmp/en_m3e_test_repeat_a");
    const auto second = m3e_test::run("/tmp/en_m3e_test_repeat_b");
    m3e_test::expect(first.ok && second.ok, "both deterministic runs complete");
    m3e_test::expect(first.termination_reason == second.termination_reason &&
                         first.exploration.selected_goals ==
                             second.exploration.selected_goals &&
                         first.exploration.completed_goals ==
                             second.exploration.completed_goals &&
                         first.exploration.plan_count ==
                             second.exploration.plan_count &&
                         first.exploration.expanded_astar_nodes ==
                             second.exploration.expanded_astar_nodes,
                     "goal order effects and planner totals repeat");
    m3e_test::expect(first.final_map_checksum ==
                         second.final_map_checksum &&
                         first.exploration.map_revision_end ==
                             second.exploration.map_revision_end,
                     "final sparse map and revision repeat");
    return 0;
}
