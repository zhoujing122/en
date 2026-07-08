#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_log_codec.hpp"

#include <sstream>
#include <vector>

namespace robot_slamd {

class RealSensorReplaySampleLog {
public:
    static std::vector<std::string> valid_log_lines() {
        return {"# m3b2 valid replay log",
                packet_line(100.0, 0.02, 100.0, 100.0, 100.0),
                packet_line(100.01, 0.02, 100.01, 100.01, 100.01),
                packet_line(100.02, 0.02, 100.02, 100.02, 100.02)};
    }

    static std::vector<std::string> invalid_latency_log_lines() {
        return {"# m3b2 invalid request latency replay log",
                packet_line(100.0, 0.30, 100.0, 100.0, 100.0)};
    }

    static std::vector<std::string> sync_too_large_log_lines() {
        return {"# m3b2 sync-too-large replay log",
                packet_line(100.075, 0.02, 100.0, 100.15, 100.075)};
    }

    static RealSensorReplayLog valid_log() {
        return codec().parse_lines(valid_log_lines(), "valid_replay_log");
    }

    static RealSensorReplayLog invalid_latency_log() {
        return codec().parse_lines(invalid_latency_log_lines(),
                                   "invalid_latency_replay_log");
    }

    static RealSensorReplayLog sync_too_large_log() {
        return codec().parse_lines(sync_too_large_log_lines(),
                                   "sync_too_large_replay_log");
    }

private:
    static RealSensorReplayLogCodec codec() {
        return RealSensorReplayLogCodec{};
    }

    static std::string packet_line(
        double packet_time_s,
        double request_latency_s,
        double tof_est_s,
        double imu_time_s,
        double wheel_est_s) {
        const double tof_req_start_s = tof_est_s - 0.5 * request_latency_s;
        const double tof_resp_s = tof_est_s + 0.5 * request_latency_s;
        const double wheel_req_start_s = wheel_est_s - 0.5 * request_latency_s;
        const double wheel_resp_s = wheel_est_s + 0.5 * request_latency_s;
        std::ostringstream o;
        o << "kind=packet"
          << ",packet_time=" << packet_time_s
          << ",tof_req_start=" << tof_req_start_s
          << ",tof_resp=" << tof_resp_s
          << ",tof_est=" << tof_est_s
          << ",tof_latency=" << request_latency_s
          << ",tof_frame=tof_frame"
          << ",tof_ranges=1.0;1.2;nan;1.5"
          << ",tof_angle_min=-0.2"
          << ",tof_angle_max=0.2"
          << ",tof_angle_inc=0.1"
          << ",tof_range_min=0.05"
          << ",tof_range_max=4.0"
          << ",imu_time=" << imu_time_s
          << ",imu_frame=imu_frame"
          << ",imu_yaw_rate=0.1"
          << ",imu_ax=0.0"
          << ",imu_ay=0.0"
          << ",imu_az=9.8"
          << ",wheel_req_start=" << wheel_req_start_s
          << ",wheel_resp=" << wheel_resp_s
          << ",wheel_est=" << wheel_est_s
          << ",wheel_latency=" << request_latency_s
          << ",wheel_frame=wheel_frame"
          << ",wheel_valid=true"
          << ",wheel_linear=0.05"
          << ",wheel_angular=0.1";
        return o.str();
    }
};

} // namespace robot_slamd
