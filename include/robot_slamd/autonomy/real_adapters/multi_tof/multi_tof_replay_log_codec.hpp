#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_types.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/scalar_tof_9byte_codec.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

class MultiTofReplayLogCodec {
public:
    std::string encode_record(const MultiTofRawPacket &packet) const {
        std::ostringstream o;
        o << "kind=multi_tof_packet"
          << " packet_time=" << packet.packet_timestamp_s
          << " packet_source=" << packet.packet_source;
        append_tof(o, "front", packet.has_front, packet.front);
        append_tof(o, "left", packet.has_left, packet.left);
        append_tof(o, "right", packet.has_right, packet.right);
        o << " imu_present=" << (packet.has_imu ? 1 : 0)
          << " imu_time=" << packet.imu.timestamp_s
          << " imu_yaw_rate=" << packet.imu.yaw_rate_rad_s
          << " imu_ax=" << packet.imu.accel_x_mps2
          << " imu_ay=" << packet.imu.accel_y_mps2
          << " imu_az=" << packet.imu.accel_z_mps2
          << " wheel_present=" << (packet.has_wheel ? 1 : 0)
          << " wheel_req_start=" << packet.wheel.timing.request_start_s
          << " wheel_resp=" << packet.wheel.timing.response_received_s
          << " wheel_est=" << packet.wheel.timing.estimated_sample_time_s
          << " wheel_latency=" << packet.wheel.timing.request_latency_s
          << " wheel_linear=" << packet.wheel.linear_velocity_mps
          << " wheel_angular=" << packet.wheel.angular_velocity_rad_s
          << " wheel_valid=" << (packet.wheel.valid ? 1 : 0);
        return o.str();
    }

    MultiTofReplayRecord decode_line(
        const std::string &line,
        int line_number) const {
        MultiTofReplayRecord record;
        record.raw_line = line;
        record.line_number = line_number;
        const auto stripped = trim_copy(line);
        if (stripped.empty() || stripped.front() == '#') {
            record.kind = MultiTofReplayRecordKind::Comment;
            return record;
        }
        const auto kv = parse_kv(stripped);
        if (!kv.count("kind")) return invalid(line, line_number, "missing_field_kind");
        if (kv.at("kind") != "multi_tof_packet") return invalid(line, line_number, "parse_error_invalid_kind");
        const char *required[] = {"packet_time", "front_present", "left_present", "right_present", "imu_present", "wheel_present"};
        for (const char *key : required) {
            if (!kv.count(key)) return invalid(line, line_number, std::string("missing_field_") + key);
        }
        bool ok = false;
        MultiTofRawPacket packet;
        packet.packet_timestamp_s = parse_double(kv, "packet_time", ok);
        if (!ok) return invalid(line, line_number, "invalid_numeric_packet_time");
        packet.packet_source = value_or(kv, "packet_source", "multi_tof_replay");
        packet.has_front = parse_bool(kv.at("front_present"), ok);
        if (!ok) return invalid(line, line_number, "invalid_bool_front_present");
        packet.has_left = parse_bool(kv.at("left_present"), ok);
        if (!ok) return invalid(line, line_number, "invalid_bool_left_present");
        packet.has_right = parse_bool(kv.at("right_present"), ok);
        if (!ok) return invalid(line, line_number, "invalid_bool_right_present");
        packet.has_imu = parse_bool(kv.at("imu_present"), ok);
        if (!ok) return invalid(line, line_number, "invalid_bool_imu_present");
        packet.has_wheel = parse_bool(kv.at("wheel_present"), ok);
        if (!ok) return invalid(line, line_number, "invalid_bool_wheel_present");
        if (packet.has_front && !parse_tof(kv, "front", MultiTofMountId::Front, packet.front, record)) return record;
        if (packet.has_left && !parse_tof(kv, "left", MultiTofMountId::Left, packet.left, record)) return record;
        if (packet.has_right && !parse_tof(kv, "right", MultiTofMountId::Right, packet.right, record)) return record;
        if (packet.has_imu) {
            const char *imu_required[] = {"imu_time", "imu_yaw_rate", "imu_ax", "imu_ay", "imu_az"};
            for (const char *key : imu_required) if (!kv.count(key)) return invalid(line, line_number, std::string("missing_field_") + key);
            packet.imu.timestamp_s = parse_double(kv, "imu_time", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_imu_time");
            packet.imu.yaw_rate_rad_s = parse_double(kv, "imu_yaw_rate", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_imu_yaw_rate");
            packet.imu.accel_x_mps2 = parse_double(kv, "imu_ax", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_imu_ax");
            packet.imu.accel_y_mps2 = parse_double(kv, "imu_ay", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_imu_ay");
            packet.imu.accel_z_mps2 = parse_double(kv, "imu_az", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_imu_az");
            packet.imu.frame_id = "imu_frame";
            packet.imu.source = "multi_tof_replay_imu";
        }
        if (packet.has_wheel) {
            const char *wheel_required[] = {"wheel_req_start", "wheel_resp", "wheel_est", "wheel_latency", "wheel_linear", "wheel_angular", "wheel_valid"};
            for (const char *key : wheel_required) if (!kv.count(key)) return invalid(line, line_number, std::string("missing_field_") + key);
            packet.wheel.timing.request_start_s = parse_double(kv, "wheel_req_start", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_wheel_req_start");
            packet.wheel.timing.response_received_s = parse_double(kv, "wheel_resp", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_wheel_resp");
            packet.wheel.timing.estimated_sample_time_s = parse_double(kv, "wheel_est", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_wheel_est");
            packet.wheel.timing.request_latency_s = parse_double(kv, "wheel_latency", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_wheel_latency");
            packet.wheel.timing.timing_source = "multi_tof_replay_request_window";
            packet.wheel.linear_velocity_mps = parse_double(kv, "wheel_linear", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_wheel_linear");
            packet.wheel.angular_velocity_rad_s = parse_double(kv, "wheel_angular", ok); if (!ok) return invalid(line, line_number, "invalid_numeric_wheel_angular");
            packet.wheel.valid = parse_bool(kv.at("wheel_valid"), ok); if (!ok) return invalid(line, line_number, "invalid_bool_wheel_valid");
            packet.wheel.frame_id = "wheel_frame";
            packet.wheel.source = "multi_tof_replay_wheel";
        }
        record.kind = MultiTofReplayRecordKind::Packet;
        record.packet = packet;
        return record;
    }

    std::vector<MultiTofReplayRecord> decode_lines(const std::vector<std::string> &lines) const {
        std::vector<MultiTofReplayRecord> records;
        int line_number = 1;
        for (const auto &line : lines) records.push_back(decode_line(line, line_number++));
        return records;
    }

    std::vector<MultiTofReplayRecord> decode_text(const std::string &text) const {
        std::vector<std::string> lines;
        std::istringstream in(text);
        std::string line;
        while (std::getline(in, line)) lines.push_back(line);
        return decode_lines(lines);
    }

private:
    static void append_tof(std::ostringstream &o, const std::string &prefix, bool present, const MultiTofRawFrame &frame) {
        o << " " << prefix << "_present=" << (present ? 1 : 0);
        if (!present) return;
        ScalarTof9BytePayload payload;
        payload.echo_tag_u48 = frame.echo_tag_u48;
        payload.distance_mm = frame.distance_mm;
        payload.confidence = frame.confidence;
        o << " " << prefix << "_frame=" << frame.frame_id
          << " " << prefix << "_payload_hex=" << hex_payload(encode_scalar_tof_9byte_payload(payload))
          << " " << prefix << "_req_start=" << frame.timing.request_start_s
          << " " << prefix << "_resp=" << frame.timing.response_received_s
          << " " << prefix << "_yaw=" << frame.mount_yaw_rad
          << " " << prefix << "_full_fov=" << frame.full_fov_rad
          << " " << prefix << "_seq=" << frame.sequence;
    }

    static bool parse_tof(const std::map<std::string, std::string> &kv, const std::string &prefix,
                          MultiTofMountId mount_id, MultiTofRawFrame &frame, MultiTofReplayRecord &record) {
        const char *suffixes[] = {"frame", "payload_hex", "req_start", "resp", "yaw", "full_fov", "seq"};
        for (const char *suffix : suffixes) {
            const auto key = prefix + "_" + suffix;
            if (!kv.count(key)) return mark_invalid(record, "missing_field_" + key);
        }
        const auto decoded = decode_scalar_tof_9byte_hex(kv.at(prefix + "_payload_hex"));
        if (!decoded.ok) return mark_invalid(record, prefix + "_" + decoded.reason);
        bool ok = false;
        frame.mount_id = mount_id;
        frame.echo_tag_u48 = decoded.payload.echo_tag_u48;
        frame.distance_mm = decoded.payload.distance_mm;
        frame.confidence = decoded.payload.confidence;
        frame.frame_id = kv.at(prefix + "_frame");
        const double req_start = parse_double(kv, prefix + "_req_start", ok); if (!ok) return mark_invalid(record, "invalid_numeric_" + prefix + "_req_start");
        const double resp = parse_double(kv, prefix + "_resp", ok); if (!ok) return mark_invalid(record, "invalid_numeric_" + prefix + "_resp");
        frame.timing = make_request_timing(req_start, resp, "multi_tof_replay_request_window");
        frame.mount_yaw_rad = parse_double(kv, prefix + "_yaw", ok); if (!ok) return mark_invalid(record, "invalid_numeric_" + prefix + "_yaw");
        frame.full_fov_rad = parse_double(kv, prefix + "_full_fov", ok); if (!ok) return mark_invalid(record, "invalid_numeric_" + prefix + "_full_fov");
        frame.sequence = parse_int(kv, prefix + "_seq", ok); if (!ok) return mark_invalid(record, "invalid_numeric_" + prefix + "_seq");
        frame.source = "multi_tof_replay_" + prefix;
        return true;
    }

    static bool mark_invalid(MultiTofReplayRecord &record, const std::string &error) {
        record.kind = MultiTofReplayRecordKind::InvalidRecord;
        record.error = error;
        return false;
    }

    static MultiTofReplayRecord invalid(const std::string &line, int line_number, const std::string &error) {
        MultiTofReplayRecord record;
        record.kind = MultiTofReplayRecordKind::InvalidRecord;
        record.raw_line = line;
        record.line_number = line_number;
        record.error = error;
        return record;
    }

    static std::map<std::string, std::string> parse_kv(const std::string &line) {
        std::map<std::string, std::string> kv;
        std::istringstream in(line);
        std::string token;
        while (in >> token) {
            const auto pos = token.find('=');
            if (pos != std::string::npos) kv[token.substr(0, pos)] = token.substr(pos + 1);
        }
        return kv;
    }

    static std::string value_or(const std::map<std::string, std::string> &kv, const std::string &key, const std::string &fallback) {
        const auto it = kv.find(key);
        return it == kv.end() ? fallback : it->second;
    }

    static double parse_double(const std::map<std::string, std::string> &kv, const std::string &key, bool &ok) {
        ok = false;
        const auto it = kv.find(key);
        if (it == kv.end()) return 0.0;
        char *end = nullptr;
        errno = 0;
        const double value = std::strtod(it->second.c_str(), &end);
        ok = end != it->second.c_str() && *end == '\0' && errno != ERANGE && std::isfinite(value);
        return ok ? value : 0.0;
    }

    static int parse_int(const std::map<std::string, std::string> &kv, const std::string &key, bool &ok) {
        ok = false;
        const auto it = kv.find(key);
        if (it == kv.end()) return 0;
        char *end = nullptr;
        errno = 0;
        const long value = std::strtol(it->second.c_str(), &end, 10);
        ok = end != it->second.c_str() && *end == '\0' && errno != ERANGE;
        return ok ? static_cast<int>(value) : 0;
    }

    static bool parse_bool(const std::string &text, bool &ok) {
        if (text == "1" || text == "true") { ok = true; return true; }
        if (text == "0" || text == "false") { ok = true; return false; }
        ok = false;
        return false;
    }

    static std::string hex_payload(const std::array<uint8_t, 9> &bytes) {
        std::ostringstream out;
        out << std::uppercase << std::hex << std::setfill('0');
        for (uint8_t byte : bytes) out << std::setw(2) << static_cast<int>(byte);
        return out.str();
    }

    static std::string trim_copy(const std::string &text) {
        const auto begin = text.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos) return "";
        const auto end = text.find_last_not_of(" \t\r\n");
        return text.substr(begin, end - begin + 1);
    }
};

} // namespace robot_slamd
