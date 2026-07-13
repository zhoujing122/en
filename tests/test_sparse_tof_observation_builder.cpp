#include "robot_slamd/mapping/sparse_tof/sparse_tof_observation_builder.hpp"
#include "robot_slamd/simulation/sensors/sim_three_scalar_tof.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ScalarTofSnapshotFrame valid_frame() {
    robot_slamd::ScalarTofSnapshotFrame frame;
    frame.distance_mm = 1500;
    frame.distance_m = 1.5;
    frame.confidence = 100;
    frame.echo_tag_u48 = 0x1234;
    frame.effective_timestamp_s = 100.0;
    frame.protocol_valid = true;
    frame.usable_for_mapping = true;
    frame.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
    frame.mount_yaw_rad = 0.5;
    frame.frame_id = "tof_left_frame";
    frame.source = "unit_test";
    return frame;
}
}

int main() {
    using namespace robot_slamd;
    SparseTofObservationBuildInput input;
    input.frame = valid_frame();
    input.estimated_pose.x_m = 1.0;
    input.estimated_pose.y_m = -2.0;
    input.estimated_pose.yaw_rad = 0.25;
    input.now_s = 100.0;

    auto hit = SparseTofObservationBuilder{}.build(input);
    expect(hit.ok, "hit observation ok");
    expect(hit.observation.return_kind == ScalarTofReturnKind::Hit,
           "hit return kind");
    expect(std::fabs(hit.observation.sensor_origin_x_m - 1.0) < 1e-12,
           "origin x from estimated pose");
    expect(std::fabs(hit.observation.sensor_origin_y_m + 2.0) < 1e-12,
           "origin y from estimated pose");
    expect(std::fabs(hit.observation.ray_yaw_rad - 0.75) < 1e-12,
           "ray yaw pose plus mount");
    expect(hit.observation.echo_tag_u48 == 0x1234,
           "echo tag preserved");
    expect(hit.observation.confidence == 100,
           "confidence preserved");
    expect(hit.observation.effective_timestamp_s == 100.0,
           "timestamp preserved");

    input.explicit_return_kind = ScalarTofReturnKind::NoReturn;
    input.frame.distance_mm = 12000;
    input.frame.distance_m = 12.0;
    auto no_return = SparseTofObservationBuilder{}.build(input);
    expect(no_return.ok, "no return observation ok");
    expect(no_return.observation.return_kind == ScalarTofReturnKind::NoReturn,
           "explicit no return kind");
    expect(no_return.observation.free_space_range_m == 12.0,
           "no return free space range");

    input.frame.confidence = 0;
    input.frame.usable_for_mapping = false;
    auto invalid = SparseTofObservationBuilder{}.build(input);
    expect(!invalid.ok, "invalid observation rejected");
    expect(invalid.observation.return_kind == ScalarTofReturnKind::Invalid,
           "invalid return kind");

    SimThreeScalarTofConfig tof_config;
    tof_config.no_hit_as_explicit_no_return = true;
    SimThreeScalarTof sim_tof{tof_config};
    FakeWorld2D empty_world;
    SimRobotState state;
    auto sample = sim_tof.sample(empty_world, state, 100.0);
    expect(sample.ok, "sim no return sample ok");
    expect(sample.packet.front.confidence == 100,
           "sim explicit no return uses valid confidence");
    expect(sample.packet.front.source.find("explicit_no_return") !=
               std::string::npos,
           "sim no return explicit metadata");

    SparseTofObservationBuildInput sim_input;
    sim_input.frame.distance_mm = sample.packet.front.distance_mm;
    sim_input.frame.distance_m =
        static_cast<double>(sample.packet.front.distance_mm) / 1000.0;
    sim_input.frame.confidence = sample.packet.front.confidence;
    sim_input.frame.echo_tag_u48 = sample.packet.front.echo_tag_u48;
    sim_input.frame.effective_timestamp_s =
        sample.packet.front.timing.estimated_sample_time_s;
    sim_input.frame.protocol_valid = true;
    sim_input.frame.usable_for_mapping = true;
    sim_input.frame.mount_yaw_rad = sample.packet.front.mount_yaw_rad;
    sim_input.frame.source = sample.packet.front.source;
    sim_input.now_s = 100.0;
    auto sim_obs = SparseTofObservationBuilder{}.build(sim_input);
    expect(sim_obs.observation.return_kind == ScalarTofReturnKind::NoReturn,
           "sim explicit metadata maps to no return");

    SimThreeScalarTof legacy_tof{};
    auto legacy = legacy_tof.sample(empty_world, state, 100.0);
    sim_input.frame.confidence = legacy.packet.front.confidence;
    sim_input.frame.source = legacy.packet.front.source;
    auto legacy_obs = SparseTofObservationBuilder{}.build(sim_input);
    expect(legacy_obs.observation.return_kind == ScalarTofReturnKind::Invalid,
           "legacy D0 confidence zero no hit remains invalid");

    return failures ? 1 : 0;
}
