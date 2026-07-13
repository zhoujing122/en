#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_types.hpp"
#include "robot_slamd/simulation/robot/sim_robot_plant.hpp"
#include "robot_slamd/simulation/world/fake_world_2d.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

namespace robot_slamd {

struct SimThreeScalarTofConfig {
    double min_range_m = 0.02;
    double max_range_m = 12.0;
    double full_fov_deg = 1.6;
    double front_latency_s = 0.004;
    double left_latency_s = 0.004;
    double right_latency_s = 0.004;
    double left_response_offset_s = 0.0002;
    double right_response_offset_s = 0.0004;
    uint8_t hit_confidence = 100;
    uint8_t no_hit_confidence = 0;
    bool no_hit_as_explicit_no_return = false;
    bool front_dropout = false;
    bool left_dropout = false;
    bool right_dropout = false;
    bool low_confidence_front = false;
    double injected_pairwise_skew_s = 0.0;
};

struct SimThreeScalarTofSample {
    bool ok = false;
    MultiTofRawPacket packet;
    int front_count = 0;
    int left_count = 0;
    int right_count = 0;
    bool any_no_hit = false;
    std::string message;
};

class SimThreeScalarTof {
public:
    explicit SimThreeScalarTof(SimThreeScalarTofConfig config = {})
        : config_(config) {}

    bool valid() const {
        return sim_finite(config_.min_range_m) &&
               sim_finite(config_.max_range_m) &&
               config_.min_range_m >= 0.0 &&
               config_.max_range_m > config_.min_range_m &&
               config_.max_range_m <= 12.0 &&
               sim_finite(config_.full_fov_deg) &&
               config_.full_fov_deg > 0.0 &&
               sim_finite(config_.front_latency_s) &&
               sim_finite(config_.left_latency_s) &&
               sim_finite(config_.right_latency_s) &&
               config_.front_latency_s >= 0.0 &&
               config_.left_latency_s >= 0.0 &&
               config_.right_latency_s >= 0.0 &&
               sim_finite(config_.left_response_offset_s) &&
               sim_finite(config_.right_response_offset_s) &&
               sim_finite(config_.injected_pairwise_skew_s);
    }

    SimThreeScalarTofSample sample(const FakeWorld2D &world,
                                   const SimRobotState &state,
                                   double now_s) {
        SimThreeScalarTofSample sample;
        if (!valid() || !sim_finite(now_s)) {
            sample.message = "sim_tof_invalid_config";
            return sample;
        }
        sample.packet.packet_timestamp_s = now_s;
        sample.packet.packet_source = "sim_three_scalar_tof";
        if (!config_.front_dropout) {
            sample.packet.front = make_frame(world, state, now_s, MultiTofMountId::Front,
                                             "tof_front_frame", 0.0,
                                             config_.front_latency_s, 0.0,
                                             0x100000000000ULL, front_sequence_++);
            if (config_.low_confidence_front) {
                sample.packet.front.confidence = 69;
            }
            sample.packet.has_front = true;
            sample.front_count = 1;
            sample.any_no_hit = sample.any_no_hit || sample.packet.front.confidence == 0;
        }
        if (!config_.left_dropout) {
            sample.packet.left = make_frame(world, state, now_s, MultiTofMountId::Left,
                                            "tof_left_frame", kSimPi * 0.5,
                                            config_.left_latency_s,
                                            config_.left_response_offset_s +
                                                config_.injected_pairwise_skew_s,
                                            0x200000000000ULL, left_sequence_++);
            sample.packet.has_left = true;
            sample.left_count = 1;
            sample.any_no_hit = sample.any_no_hit || sample.packet.left.confidence == 0;
        }
        if (!config_.right_dropout) {
            sample.packet.right = make_frame(world, state, now_s, MultiTofMountId::Right,
                                             "tof_right_frame", -kSimPi * 0.5,
                                             config_.right_latency_s,
                                             config_.right_response_offset_s,
                                             0x300000000000ULL, right_sequence_++);
            sample.packet.has_right = true;
            sample.right_count = 1;
            sample.any_no_hit = sample.any_no_hit || sample.packet.right.confidence == 0;
        }
        sample.ok = sample.packet.has_front || sample.packet.has_left || sample.packet.has_right;
        sample.message = sample.ok ? "sim_three_scalar_tof_ok" : "sim_three_scalar_tof_dropout";
        return sample;
    }

private:
    MultiTofRawFrame make_frame(const FakeWorld2D &world,
                                const SimRobotState &state,
                                double now_s,
                                MultiTofMountId mount_id,
                                const std::string &frame_id,
                                double mount_yaw_rad,
                                double latency_s,
                                double response_offset_s,
                                uint64_t echo_prefix,
                                int sequence) const {
        const double ray_yaw = sim_normalize_yaw(state.pose.yaw_rad + mount_yaw_rad);
        const auto hit = world.raycast(state.pose.x_m, state.pose.y_m, ray_yaw,
                                       config_.min_range_m, config_.max_range_m);
        const double distance_m = hit.hit ? hit.distance_m : config_.max_range_m;
        const auto distance_mm = static_cast<uint16_t>(
            sim_clamp(std::round(distance_m * 1000.0), 0.0, 12000.0));
        MultiTofRawFrame frame;
        frame.mount_id = mount_id;
        frame.echo_tag_u48 = (echo_prefix + static_cast<uint64_t>(sequence)) &
                             0x0000FFFFFFFFFFFFULL;
        frame.distance_mm = distance_mm;
        frame.confidence = hit.hit || config_.no_hit_as_explicit_no_return
                               ? config_.hit_confidence
                               : config_.no_hit_confidence;
        const double response = now_s - response_offset_s;
        frame.timing = make_request_timing(response - latency_s,
                                           response,
                                           "sim_tof_request_midpoint");
        frame.frame_id = frame_id;
        frame.mount_yaw_rad = mount_yaw_rad;
        frame.full_fov_rad = sim_degrees_to_radians(config_.full_fov_deg);
        frame.sequence = sequence;
        frame.source = hit.hit ? "sim_center_raycast_hit"
                               : (config_.no_hit_as_explicit_no_return
                                      ? "sim_center_raycast_explicit_no_return"
                                      : "sim_center_raycast_no_hit");
        return frame;
    }

    SimThreeScalarTofConfig config_;
    int front_sequence_ = 1;
    int left_sequence_ = 1;
    int right_sequence_ = 1;
};

} // namespace robot_slamd
