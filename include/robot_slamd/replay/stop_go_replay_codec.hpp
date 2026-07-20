#pragma once

#include "robot_slamd/mapping/stop_go_mapping_controller.hpp"

#include <cstdint>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace robot_slamd {

inline int stop_go_state_code(RelativeMotionStepState state) {
    return static_cast<int>(state);
}

inline RelativeMotionStepState stop_go_state_from_code(int value) {
    if (value < static_cast<int>(RelativeMotionStepState::Accepted) ||
        value > static_cast<int>(RelativeMotionStepState::EstopLatched)) {
        return RelativeMotionStepState::Fault;
    }
    return static_cast<RelativeMotionStepState>(value);
}

inline int stop_go_action_code(RelativeMotionStepAction action) {
    return static_cast<int>(action);
}

inline RelativeMotionStepAction stop_go_action_from_code(int value) {
    if (value < 0 || value > 5) return RelativeMotionStepAction::Stop;
    return static_cast<RelativeMotionStepAction>(value);
}

struct StopGoReplayCodecResult {
    bool ok = false;
    std::string reason = "not_decoded";
    StopGoReplayRecord record;
};

class StopGoReplayLogCodec final {
public:
    static std::string encode(const StopGoReplayRecord &record) {
        const auto &s = record.snapshot;
        const auto &st = record.stable_sample;
        std::ostringstream out;
        out << std::setprecision(17)
            << "cycle=" << record.cycle_index
            << " command_id=" << record.command_id
            << " action=" << stop_go_action_code(record.motion.action)
            << " state=" << stop_go_state_code(record.motion.state)
            << " requested=" << record.motion.requested_amount
            << " actual_distance=" << record.motion.actual_distance_mm
            << " actual_angle=" << record.motion.actual_angle_deg
            << " error_code=" << record.motion.error_code
            << " reason=" << (record.motion.reason.empty() ? "none" : record.motion.reason)
            << " wheel_t=" << s.wheel.timestamp_s
            << " linear=" << s.wheel.linear_mps
            << " angular=" << s.wheel.angular_rad_s
            << " left_rpm=" << s.wheel.left_rpm
            << " right_rpm=" << s.wheel.right_rpm
            << " left_ticks=" << s.wheel.left_ticks
            << " right_ticks=" << s.wheel.right_ticks
            << " pair_skew=" << s.wheel.pair_skew_s
            << " wheel_valid=" << (s.wheel.valid ? 1 : 0)
            << " imu_t=" << s.imu.timestamp_s
            << " gyro_z=" << s.imu.yaw_rate_rad_s
            << " map_before=" << record.map_revision_before
            << " map_after=" << record.map_revision_after
            << " motor_settled_us=" << record.motor_settled_timestamp_us
            << " mapping_ready_us=" << record.mapping_ready_timestamp_us
            << " settle_frames=" << record.settle_frame_count
            << " settle_ms=" << record.settle_duration_ms;
        append_tof(out, "front", st.front, s.multi_tof.front);
        append_tof(out, "left", st.left, s.multi_tof.left);
        append_tof(out, "right", st.right, s.multi_tof.right);
        out << " epoch=" << record.frame_transform_epoch
            << " wall_point_accepted=" << (record.left_wall_point_accepted ? 1 : 0)
            << " wall_point_x=" << record.left_wall_point.x_m
            << " wall_point_y=" << record.left_wall_point.y_m
            << " wall_point_t=" << record.left_wall_point.timestamp_s
            << " wall_point_confidence=" << record.left_wall_point.confidence
            << " wall_point_cycle=" << record.left_wall_point.cycle_index
            << " wall_point_revision=" << record.left_wall_point.map_revision
            << " wall_point_segment=" << record.left_wall_point.wall_segment_id
            << " wall_model_valid=" << (record.wall_model.valid ? 1 : 0)
            << " wall_heading=" << record.wall_model.wall_heading_rad
            << " wall_distance=" << record.wall_model.signed_base_to_wall_distance_m
            << " wall_rms=" << record.wall_model.rms_residual_m
            << " wall_baseline=" << record.wall_model.baseline_m
            << " wall_input=" << record.wall_model.input_point_count
            << " wall_inliers=" << record.wall_model.inlier_point_count
            << " wall_model_segment=" << record.wall_model.wall_segment_id
            << " control_action=" << static_cast<int>(record.control_decision.action)
            << " heading_error=" << record.control_decision.heading_error_rad
            << " distance_error=" << record.control_decision.distance_error_m
            << " distance_bias=" << record.control_decision.distance_bias_rad
            << " turn_error=" << record.control_decision.turn_error_rad
            << " correction_deg=" << record.control_decision.correction_amount_deg
            << " post_turn_verified=" << (record.post_turn_verified ? 1 : 0)
            << " corner_candidate=" << (record.corner_candidate ? 1 : 0)
            << " corner_confirmation=" << (record.corner_confirmation ? 1 : 0)
            << " corner_confirmed=" << (record.corner_confirmed ? 1 : 0)
            << " corner_clearance=" << (record.corner_right_clearance_passed ? 1 : 0)
            << " corner_main_turn=" << (record.corner_main_turn ? 1 : 0)
            << " corner_residual=" << (record.corner_residual_correction ? 1 : 0)
            << " corner_residual_right=" << (record.corner_residual_right ? 1 : 0)
            << " post_corner_sensor_verified=" << (record.post_corner_sensor_verified ? 1 : 0)
            << " new_wall_reacquisition=" << (record.new_wall_reacquisition ? 1 : 0)
            << " post_corner_follow_steps=" << record.post_corner_follow_steps
            << " corner_confirmation_samples=" << record.corner_confirmation_samples
            << " corner_confirmation_rejects=" << record.corner_confirmation_rejects
            << " corner_right_clearance_checks=" << record.corner_right_clearance_checks
            << " wall_segment_id=" << record.wall_segment_id
            << " corner_transition_id=" << record.corner_transition_id
            << " corner_main_command_id=" << record.corner_main_command_id
            << " corner_residual_command_id=" << record.corner_residual_command_id
            << " pre_turn_odom_yaw=" << record.pre_turn_odom_yaw_rad
            << " post_turn_odom_yaw=" << record.post_turn_odom_yaw_rad
            << " actual_turn_delta=" << record.actual_turn_delta_rad
            << " turn_residual=" << record.turn_residual_rad
            << " odom_count=" << record.odom_samples.size();
        for (std::size_t index = 0; index < record.odom_samples.size(); ++index) {
            const auto &sample = record.odom_samples[index];
            const auto key = [index](const char *suffix) {
                std::ostringstream name;
                name << "odom" << index << "_" << suffix;
                return name.str();
            };
            out << " " << key("wheel_t") << "=" << sample.wheel.timestamp_s
                << " " << key("linear") << "=" << sample.wheel.linear_mps
                << " " << key("angular") << "=" << sample.wheel.angular_rad_s
                << " " << key("left_ticks") << "=" << sample.wheel.left_ticks
                << " " << key("right_ticks") << "=" << sample.wheel.right_ticks
                << " " << key("wheel_valid") << "=" << (sample.wheel.valid ? 1 : 0)
                << " " << key("imu_t") << "=" << sample.imu.timestamp_s
                << " " << key("gyro_z") << "=" << sample.imu.yaw_rate_rad_s;
        }
        out << " controller_state=" << static_cast<int>(record.controller_state) << "\n";
        return out.str();
    }

    static StopGoReplayCodecResult decode(const std::string &line) {
        StopGoReplayCodecResult result;
        std::istringstream input(line);
        std::map<std::string, std::string> fields;
        std::string token;
        while (input >> token) {
            const auto pos = token.find('=');
            if (pos == std::string::npos) {
                result.reason = "replay_token_missing_equals";
                return result;
            }
            fields[token.substr(0, pos)] = token.substr(pos + 1);
        }
        try {
            result.record.cycle_index = static_cast<std::size_t>(std::stoull(fields.at("cycle")));
            result.record.command_id = std::stoull(fields.at("command_id"));
            result.record.motion.command_id = result.record.command_id;
            result.record.motion.action = stop_go_action_from_code(std::stoi(fields.at("action")));
            result.record.motion.state = stop_go_state_from_code(std::stoi(fields.at("state")));
            result.record.motion.requested_amount = std::stod(fields.at("requested"));
            result.record.motion.actual_distance_mm = std::stod(fields.at("actual_distance"));
            result.record.motion.actual_angle_deg = std::stod(fields.at("actual_angle"));
            result.record.motion.error_code = std::stoi(fields.at("error_code"));
            result.record.motion.reason = fields.at("reason");
            auto &s = result.record.snapshot;
            s.has_wheel = true;
            s.wheel.timestamp_s = std::stod(fields.at("wheel_t"));
            s.wheel.linear_mps = std::stod(fields.at("linear"));
            s.wheel.angular_rad_s = std::stod(fields.at("angular"));
            s.wheel.left_rpm = std::stod(fields.at("left_rpm"));
            s.wheel.right_rpm = std::stod(fields.at("right_rpm"));
            s.wheel.left_ticks = std::stod(fields.at("left_ticks"));
            s.wheel.right_ticks = std::stod(fields.at("right_ticks"));
            s.wheel.pair_skew_s = std::stod(fields.at("pair_skew"));
            s.wheel.pair_skew_valid = true;
            s.wheel.valid = std::stoi(fields.at("wheel_valid")) != 0;
            s.has_imu = true;
            s.imu.timestamp_s = std::stod(fields.at("imu_t"));
            s.imu.yaw_rate_rad_s = std::stod(fields.at("gyro_z"));
            result.record.map_revision_before = std::stoull(fields.at("map_before"));
            result.record.map_revision_after = std::stoull(fields.at("map_after"));
            result.record.motor_settled_timestamp_us = std::stoull(fields.at("motor_settled_us"));
            result.record.mapping_ready_timestamp_us = std::stoull(fields.at("mapping_ready_us"));
            result.record.settle_frame_count = std::stoull(fields.at("settle_frames"));
            result.record.settle_duration_ms = std::stod(fields.at("settle_ms"));
            decode_tof(fields, "front", result.record.stable_sample.front, s.multi_tof.front);
            decode_tof(fields, "left", result.record.stable_sample.left, s.multi_tof.left);
            decode_tof(fields, "right", result.record.stable_sample.right, s.multi_tof.right);
            read_optional(fields, "epoch", result.record.frame_transform_epoch);
            read_optional(fields, "wall_point_accepted", result.record.left_wall_point_accepted);
            read_optional(fields, "wall_point_x", result.record.left_wall_point.x_m);
            read_optional(fields, "wall_point_y", result.record.left_wall_point.y_m);
            read_optional(fields, "wall_point_t", result.record.left_wall_point.timestamp_s);
            read_optional(fields, "wall_point_confidence", result.record.left_wall_point.confidence);
            read_optional(fields, "wall_point_cycle", result.record.left_wall_point.cycle_index);
            read_optional(fields, "wall_point_revision", result.record.left_wall_point.map_revision);
            read_optional(fields, "wall_point_segment", result.record.left_wall_point.wall_segment_id);
            read_optional(fields, "wall_model_valid", result.record.wall_model.valid);
            read_optional(fields, "wall_heading", result.record.wall_model.wall_heading_rad);
            read_optional(fields, "wall_distance", result.record.wall_model.signed_base_to_wall_distance_m);
            read_optional(fields, "wall_rms", result.record.wall_model.rms_residual_m);
            read_optional(fields, "wall_baseline", result.record.wall_model.baseline_m);
            read_optional(fields, "wall_input", result.record.wall_model.input_point_count);
            read_optional(fields, "wall_inliers", result.record.wall_model.inlier_point_count);
            read_optional(fields, "wall_model_segment", result.record.wall_model.wall_segment_id);
            int control_action = 0;
            read_optional(fields, "control_action", control_action);
            if (control_action >= 0 && control_action <= 2) {
                result.record.control_decision.action =
                    static_cast<LeftWallControlAction>(control_action);
            }
            read_optional(fields, "heading_error", result.record.control_decision.heading_error_rad);
            read_optional(fields, "distance_error", result.record.control_decision.distance_error_m);
            read_optional(fields, "distance_bias", result.record.control_decision.distance_bias_rad);
            read_optional(fields, "turn_error", result.record.control_decision.turn_error_rad);
            read_optional(fields, "correction_deg", result.record.control_decision.correction_amount_deg);
            read_optional(fields, "post_turn_verified", result.record.post_turn_verified);
            read_optional(fields, "corner_candidate", result.record.corner_candidate);
            read_optional(fields, "corner_confirmation", result.record.corner_confirmation);
            read_optional(fields, "corner_confirmed", result.record.corner_confirmed);
            read_optional(fields, "corner_clearance", result.record.corner_right_clearance_passed);
            read_optional(fields, "corner_main_turn", result.record.corner_main_turn);
            read_optional(fields, "corner_residual", result.record.corner_residual_correction);
            read_optional(fields, "corner_residual_right", result.record.corner_residual_right);
            read_optional(fields, "post_corner_sensor_verified", result.record.post_corner_sensor_verified);
            read_optional(fields, "new_wall_reacquisition", result.record.new_wall_reacquisition);
            read_optional(fields, "post_corner_follow_steps", result.record.post_corner_follow_steps);
            read_optional(fields, "corner_confirmation_samples", result.record.corner_confirmation_samples);
            read_optional(fields, "corner_confirmation_rejects", result.record.corner_confirmation_rejects);
            read_optional(fields, "corner_right_clearance_checks", result.record.corner_right_clearance_checks);
            read_optional(fields, "wall_segment_id", result.record.wall_segment_id);
            read_optional(fields, "corner_transition_id", result.record.corner_transition_id);
            read_optional(fields, "corner_main_command_id", result.record.corner_main_command_id);
            read_optional(fields, "corner_residual_command_id", result.record.corner_residual_command_id);
            read_optional(fields, "pre_turn_odom_yaw", result.record.pre_turn_odom_yaw_rad);
            read_optional(fields, "post_turn_odom_yaw", result.record.post_turn_odom_yaw_rad);
            read_optional(fields, "actual_turn_delta", result.record.actual_turn_delta_rad);
            read_optional(fields, "turn_residual", result.record.turn_residual_rad);
            std::size_t odom_count = 0;
            read_optional(fields, "odom_count", odom_count);
            if (odom_count > 10000U) {
                result.reason = "replay_odom_sample_count_exceeded";
                return result;
            }
            for (std::size_t index = 0; index < odom_count; ++index) {
                const auto key = [index](const char *suffix) {
                    std::ostringstream name;
                    name << "odom" << index << "_" << suffix;
                    return name.str();
                };
                StopGoReplayOdomSample sample;
                sample.wheel.timestamp_s = std::stod(fields.at(key("wheel_t")));
                sample.wheel.linear_mps = std::stod(fields.at(key("linear")));
                sample.wheel.angular_rad_s = std::stod(fields.at(key("angular")));
                sample.wheel.left_ticks = std::stod(fields.at(key("left_ticks")));
                sample.wheel.right_ticks = std::stod(fields.at(key("right_ticks")));
                sample.wheel.valid = std::stoi(fields.at(key("wheel_valid"))) != 0;
                sample.imu.timestamp_s = std::stod(fields.at(key("imu_t")));
                sample.imu.yaw_rate_rad_s = std::stod(fields.at(key("gyro_z")));
                result.record.odom_samples.push_back(sample);
            }
            s.has_multi_tof = true;
            s.multi_tof.has_front = true;
            s.multi_tof.has_left = true;
            s.multi_tof.has_right = true;
            s.multi_tof.valid_tof_count = 3;
            s.multi_tof.synchronized_time_s = std::max({
                s.multi_tof.front.effective_timestamp_s,
                s.multi_tof.left.effective_timestamp_s,
                s.multi_tof.right.effective_timestamp_s});
            result.record.motion.final_feedback = s;
            result.ok = result.record.command_id == 0 ||
                        result.record.motion.command_id == result.record.command_id;
            result.reason = result.ok ? "replay_record_decoded" : "replay_command_id_mismatch";
        } catch (const std::exception &) {
            result.reason = "replay_record_parse_failed";
        }
        return result;
    }

    static bool write(const std::string &path,
                      const std::vector<StopGoReplayRecord> &records,
                      std::string &reason) {
        std::ofstream output(path, std::ios::out | std::ios::trunc);
        if (!output) { reason = "replay_log_open_failed"; return false; }
        for (const auto &record : records) output << encode(record);
        if (!output) { reason = "replay_log_write_failed"; return false; }
        reason = "replay_log_written";
        return true;
    }

    static bool read(const std::string &path,
                     std::vector<StopGoReplayRecord> &records,
                     std::string &reason) {
        std::ifstream input(path);
        if (!input) { reason = "replay_log_open_failed"; return false; }
        std::string line;
        while (std::getline(input, line)) {
            if (line.empty()) continue;
            const auto decoded = decode(line);
            if (!decoded.ok) { reason = decoded.reason; return false; }
            records.push_back(decoded.record);
        }
        reason = records.empty() ? "replay_log_empty" : "replay_log_read";
        return !records.empty();
    }

private:
    template <typename T>
    static void read_optional(const std::map<std::string, std::string> &fields,
                              const char *name, T &value) {
        const auto found = fields.find(name);
        if (found == fields.end()) return;
        if constexpr (std::is_same<T, bool>::value) {
            value = std::stoi(found->second) != 0;
        } else if constexpr (std::is_integral<T>::value) {
            value = static_cast<T>(std::stoull(found->second));
        } else {
            value = static_cast<T>(std::stod(found->second));
        }
    }

    static void append_tof(std::ostringstream &out, const char *prefix,
                           const StableTofReading &stable,
                           const ScalarTofSnapshotFrame &selected) {
        out << " " << prefix << "_mm=" << stable.distance_mm
            << " " << prefix << "_confidence=" << static_cast<int>(stable.confidence)
            << " " << prefix << "_selected_us=" << stable.sample_timestamp_us
            << " " << prefix << "_valid_count=" << stable.valid_sample_count
            << " " << prefix << "_valid=" << (stable.valid ? 1 : 0)
            << " " << prefix << "_usable=" << (stable.usable_for_mapping ? 1 : 0)
            << " " << prefix << "_raw_count=" << stable.raw_frame_count;
        for (std::size_t index = 0; index < stable.raw_frame_count; ++index) {
            const auto &raw = stable.raw_frames[index];
            out << " " << prefix << "_raw" << index << "_mm=" << raw.frame.distance_mm
                << " " << prefix << "_raw" << index << "_confidence="
                << static_cast<int>(raw.frame.confidence)
                << " " << prefix << "_raw" << index << "_valid="
                << (raw.frame.valid ? 1 : 0)
                << " " << prefix << "_raw" << index << "_req_us="
                << raw.request_start_us
                << " " << prefix << "_raw" << index << "_resp_us="
                << raw.response_received_us
                << " " << prefix << "_raw" << index << "_sample_us="
                << raw.sample_timestamp_us;
        }
        out
            << " " << prefix << "_req_us="
            << (stable.raw_frame_count ? stable.raw_frames[0].request_start_us : 0)
            << " " << prefix << "_resp_us="
            << (stable.raw_frame_count ? stable.raw_frames[0].response_received_us : 0)
            << " " << prefix << "_frame_time=" << selected.effective_timestamp_s;
    }

    static void decode_tof(const std::map<std::string, std::string> &fields,
                           const char *prefix, StableTofReading &stable,
                           ScalarTofSnapshotFrame &frame) {
        stable.distance_mm = static_cast<std::uint16_t>(std::stoul(fields.at(std::string(prefix) + "_mm")));
        stable.confidence = static_cast<std::uint8_t>(std::stoul(fields.at(std::string(prefix) + "_confidence")));
        stable.sample_timestamp_us = std::stoull(fields.at(std::string(prefix) + "_selected_us"));
        stable.valid_sample_count = std::stoull(fields.at(std::string(prefix) + "_valid_count"));
        stable.valid = std::stoi(fields.at(std::string(prefix) + "_valid")) != 0;
        stable.usable_for_mapping = std::stoi(fields.at(std::string(prefix) + "_usable")) != 0;
        stable.rejection_reason = "replay_selected_frame";
        stable.raw_frame_count = std::stoull(fields.at(std::string(prefix) + "_raw_count"));
        if (stable.raw_frame_count > stable.raw_frames.size()) {
            throw std::runtime_error("replay_raw_frame_count_exceeded");
        }
        for (std::size_t index = 0; index < stable.raw_frame_count; ++index) {
            auto &raw = stable.raw_frames[index];
            raw.frame.valid = std::stoi(fields.at(std::string(prefix) + "_raw" + std::to_string(index) + "_valid")) != 0;
            raw.frame.distance_mm = static_cast<std::uint16_t>(std::stoul(fields.at(std::string(prefix) + "_raw" + std::to_string(index) + "_mm")));
            raw.frame.confidence = static_cast<std::uint8_t>(std::stoul(fields.at(std::string(prefix) + "_raw" + std::to_string(index) + "_confidence")));
            raw.request_start_us = std::stoull(fields.at(std::string(prefix) + "_raw" + std::to_string(index) + "_req_us"));
            raw.response_received_us = std::stoull(fields.at(std::string(prefix) + "_raw" + std::to_string(index) + "_resp_us"));
            raw.sample_timestamp_us = std::stoull(fields.at(std::string(prefix) + "_raw" + std::to_string(index) + "_sample_us"));
        }
        frame.distance_mm = stable.distance_mm;
        frame.distance_m = stable.distance_mm / 1000.0;
        frame.confidence = stable.confidence;
        frame.effective_timestamp_s = std::stod(fields.at(std::string(prefix) + "_frame_time"));
        frame.protocol_valid = stable.valid;
        frame.usable_for_mapping = stable.usable_for_mapping;
        frame.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
        frame.frame_id = std::string("tof_") + prefix + "_frame";
        frame.source = "stop_go_replay";
    }
};

} // namespace robot_slamd
