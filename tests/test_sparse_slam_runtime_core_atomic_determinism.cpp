#include "m3d2c_runtime_test_helpers.hpp"

namespace {

struct Outcome {
    robot_slamd::RobotPose2D transform;
    std::uint64_t revision = 0;
    std::size_t cells = 0;
    std::size_t changed_cells = 0;
    double delta_yaw = 0.0;
};

Outcome run() {
    using namespace robot_slamd;
    auto cfg = m3d2c_test::atomic_config();
    SparseSlamRuntimeCore core(cfg);
    core.initialize(sparse_slam_initialization_request_from_config(cfg));
    (void)m3d2c_test::run_atomic_sequence(core);
    Outcome out;
    out.transform = core.frame_state().current_map_from_odom().map_T_odom;
    out.revision = core.report().current_map_revision;
    out.cells = core.report().sparse_map_cell_count;
    out.changed_cells =
        core.report().keyframe_transaction_changed_cell_count;
    out.delta_yaw = core.latest_keyframe()->matcher_delta_yaw_rad;
    return out;
}

} // namespace

int main() {
    const auto a = run();
    const auto b = run();
    m3d2a1_test::expect(a.transform.x_m == b.transform.x_m &&
                           a.transform.y_m == b.transform.y_m &&
                           a.transform.yaw_rad == b.transform.yaw_rad,
                       "atomic transform deterministic");
    m3d2a1_test::expect(a.revision == b.revision &&
                           a.cells == b.cells &&
                           a.changed_cells == b.changed_cells &&
                           a.delta_yaw == b.delta_yaw,
                       "atomic map and keyframe deterministic");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
