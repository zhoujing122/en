#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_types.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

class RealSensorReplayLogCodec {
public:
    RealSensorReplayLog parse_lines(
        const std::vector<std::string> &lines,
        const std::string &source_name = "inline_replay") const {
        RealSensorReplayLog log;
        log.source_name = source_name;
        int line_number = 1;
        for (const auto &line : lines) {
            if (trim(line).empty() || trim(line).front() == '#') {
                RealSensorReplayRecord record;
                record.kind = RealSensorReplayRecordKind::Comment;
                record.line_number = line_number;
                record.raw_line = line;
                record.message = "comment";
                log.records.push_back(record);
            } else {
                log.records.push_back(parse_packet_line(line, line_number));
            }
            line_number++;
        }
        return log;
    }

    std::vector<std::string> write_lines(
        const RealSensorReplayLog &log) const {
        std::vector<std::string> lines;
        for (const auto &record : log.records) {
            if (record.kind == RealSensorReplayRecordKind::Comment) {
                lines.push_back(record.raw_line.empty() ? "# comment"
                                                        : record.raw_line);
                continue;
            }
            if (record.kind == RealSensorReplayRecordKind::EndOfLog) {
                lines.push_back("# end_of_log");
                continue;
            }
            const auto &p = record.packet;
            std::ostringstream o;
            o << "kind=packet"
              << ",packet_time=" << p.packet_timestamp_s
              << ",tof_req_start=" << p.tof.timing.request_start_s
              << ",tof_resp=" << p.tof.timing.response_received_s
              << ",tof_est=" << p.tof.timing.estimated_sample_time_s
              << ",tof_latency=" << p.tof.timing.request_latency_s
              << ",tof_frame=" << p.tof.frame_id
              << ",tof_ranges=" << join_ranges(p.tof.ranges_m)
              << ",tof_angle_min=" << p.tof.angle_min_rad
              << ",tof_angle_max=" << p.tof.angle_max_rad
              << ",tof_angle_inc=" << p.tof.angle_increment_rad
              << ",tof_range_min=" << p.tof.range_min_m
              << ",tof_range_max=" << p.tof.range_max_m
              << ",imu_time=" << p.imu.timestamp_s
              << ",imu_frame=" << p.imu.frame_id
              << ",imu_yaw_rate=" << p.imu.yaw_rate_rad_s
              << ",imu_ax=" << p.imu.accel_x_mps2
              << ",imu_ay=" << p.imu.accel_y_mps2
              << ",imu_az=" << p.imu.accel_z_mps2
              << ",wheel_req_start=" << p.wheel.timing.request_start_s
              << ",wheel_resp=" << p.wheel.timing.response_received_s
              << ",wheel_est=" << p.wheel.timing.estimated_sample_time_s
              << ",wheel_latency=" << p.wheel.timing.request_latency_s
              << ",wheel_frame=" << p.wheel.frame_id
              << ",wheel_valid=" << (p.wheel.valid ? "true" : "false")
              << ",wheel_linear=" << p.wheel.linear_velocity_mps
              << ",wheel_angular=" << p.wheel.angular_velocity_rad_s;
            lines.push_back(o.str());
        }
        return lines;
    }

    std::string write_text(const RealSensorReplayLog &log) const {
        std::ostringstream o;
        const auto lines = write_lines(log);
        for (const auto &line : lines) {
            o << line << "\n";
        }
        return o.str();
    }

private:
    RealSensorReplayRecord parse_packet_line(
        const std::string &line,
        int line_number) const {
        RealSensorReplayRecord record;
        record.kind = RealSensorReplayRecordKind::Comment;
        record.line_number = line_number;
        record.raw_line = line;
        record.message = "parse_error";

        const auto kv = parse_kv(line);
        if (value(kv, "kind") != "packet") {
            record.message = "parse_error_missing_packet_kind";
            return record;
        }
        const char *required[] = {
            "packet_time", "tof_req_start", "tof_resp", "tof_est",
            "tof_latency", "tof_frame", "tof_ranges", "tof_angle_min",
            "tof_angle_max", "tof_angle_inc", "tof_range_min",
            "tof_range_max", "imu_time", "imu_frame", "imu_yaw_rate",
            "imu_ax", "imu_ay", "imu_az", "wheel_req_start",
            "wheel_resp", "wheel_est", "wheel_latency", "wheel_frame",
            "wheel_valid", "wheel_linear", "wheel_angular"};
        for (const char *key : required) {
            if (!kv.count(key)) {
                record.message = std::string("parse_error_missing_") + key;
                return record;
            }
        }

        record.kind = RealSensorReplayRecordKind::Packet;
        record.message = "packet";
        auto &p = record.packet;
        p.has_tof = true;
        p.has_imu = true;
        p.has_wheel = true;
        p.packet_timestamp_s = parse_double(value(kv, "packet_time"));
        p.packet_source = "sensor_replay";
        p.tof.timing.request_start_s = parse_double(value(kv, "tof_req_start"));
        p.tof.timing.response_received_s = parse_double(value(kv, "tof_resp"));
        p.tof.timing.estimated_sample_time_s = parse_double(value(kv, "tof_est"));
        p.tof.timing.request_latency_s = parse_double(value(kv, "tof_latency"));
        p.tof.timing.timing_source = "replay_request_window";
        p.tof.frame_id = value(kv, "tof_frame");
        p.tof.ranges_m = parse_ranges(value(kv, "tof_ranges"));
        p.tof.angle_min_rad = parse_double(value(kv, "tof_angle_min"));
        p.tof.angle_max_rad = parse_double(value(kv, "tof_angle_max"));
        p.tof.angle_increment_rad = parse_double(value(kv, "tof_angle_inc"));
        p.tof.range_min_m = parse_double(value(kv, "tof_range_min"));
        p.tof.range_max_m = parse_double(value(kv, "tof_range_max"));
        p.tof.source = "sensor_replay_tof";
        p.imu.timestamp_s = parse_double(value(kv, "imu_time"));
        p.imu.frame_id = value(kv, "imu_frame");
        p.imu.yaw_rate_rad_s = parse_double(value(kv, "imu_yaw_rate"));
        p.imu.accel_x_mps2 = parse_double(value(kv, "imu_ax"));
        p.imu.accel_y_mps2 = parse_double(value(kv, "imu_ay"));
        p.imu.accel_z_mps2 = parse_double(value(kv, "imu_az"));
        p.imu.source = "sensor_replay_imu";
        p.wheel.timing.request_start_s = parse_double(value(kv, "wheel_req_start"));
        p.wheel.timing.response_received_s = parse_double(value(kv, "wheel_resp"));
        p.wheel.timing.estimated_sample_time_s = parse_double(value(kv, "wheel_est"));
        p.wheel.timing.request_latency_s = parse_double(value(kv, "wheel_latency"));
        p.wheel.timing.timing_source = "replay_request_window";
        p.wheel.frame_id = value(kv, "wheel_frame");
        p.wheel.valid = parse_bool(value(kv, "wheel_valid"));
        p.wheel.linear_velocity_mps = parse_double(value(kv, "wheel_linear"));
        p.wheel.angular_velocity_rad_s = parse_double(value(kv, "wheel_angular"));
        p.wheel.source = "sensor_replay_wheel";
        return record;
    }

    static std::string trim(const std::string &s) {
        auto begin = s.begin();
        while (begin != s.end() && std::isspace(static_cast<unsigned char>(*begin))) {
            ++begin;
        }
        auto end = s.end();
        while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
            --end;
        }
        return std::string(begin, end);
    }

    static std::map<std::string, std::string> parse_kv(
        const std::string &line) {
        std::map<std::string, std::string> out;
        std::size_t start = 0;
        while (start <= line.size()) {
            const std::size_t comma = line.find(',', start);
            const std::string part = line.substr(
                start,
                comma == std::string::npos ? std::string::npos : comma - start);
            const std::size_t eq = part.find('=');
            if (eq != std::string::npos) {
                out[trim(part.substr(0, eq))] = trim(part.substr(eq + 1));
            }
            if (comma == std::string::npos) {
                break;
            }
            start = comma + 1;
        }
        return out;
    }

    static std::string value(
        const std::map<std::string, std::string> &kv,
        const std::string &key) {
        const auto it = kv.find(key);
        return it == kv.end() ? std::string() : it->second;
    }

    static double parse_double(const std::string &text) {
        std::string lower = text;
        std::transform(lower.begin(), lower.end(), lower.begin(), [](char c) {
            return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        });
        if (lower == "nan") {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::strtod(text.c_str(), nullptr);
    }

    static bool parse_bool(const std::string &text) {
        return text == "true" || text == "1" || text == "yes";
    }

    static std::vector<double> parse_ranges(const std::string &text) {
        std::vector<double> ranges;
        std::size_t start = 0;
        while (start <= text.size()) {
            const std::size_t sep = text.find(';', start);
            ranges.push_back(parse_double(text.substr(
                start,
                sep == std::string::npos ? std::string::npos : sep - start)));
            if (sep == std::string::npos) {
                break;
            }
            start = sep + 1;
        }
        return ranges;
    }

    static std::string join_ranges(const std::vector<double> &ranges) {
        std::ostringstream o;
        for (std::size_t i = 0; i < ranges.size(); ++i) {
            if (i) {
                o << ";";
            }
            if (std::isfinite(ranges[i])) {
                o << ranges[i];
            } else {
                o << "nan";
            }
        }
        return o.str();
    }
};

} // namespace robot_slamd
