#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"

#include <string>
#include <vector>
#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

int main() {
    using namespace robot_slamd;

    RealSensorReplayLogCodec codec;
    auto invalid = codec.parse_lines({"not_a_packet"});
    expect(invalid.records.front().kind == RealSensorReplayRecordKind::InvalidRecord,
           "invalid line becomes invalid record");

    auto missing_kind = codec.parse_lines({"packet_time=100.0"});
    expect(missing_kind.records.front().message == "parse_error_missing_packet_kind",
           "missing kind detected");

    auto line = RealSensorReplaySampleLog::valid_log_lines()[1];
    auto missing_field = line;
    const auto marker = std::string(",tof_resp=");
    const auto start = missing_field.find(marker);
    const auto end = missing_field.find(",tof_est=", start);
    missing_field.erase(start, end - start);
    auto missing = codec.parse_lines({missing_field});
    expect(missing.records.front().message == "parse_error_missing_tof_resp",
           "missing required field detected");

    auto invalid_double = line;
    invalid_double.replace(invalid_double.find("packet_time=100"), 15,
                           "packet_time=abc");
    auto bad_double = codec.parse_lines({invalid_double});
    expect(bad_double.records.front().message == "parse_error_invalid_number_packet_time",
           "invalid double detected");

    auto invalid_bool = line;
    invalid_bool.replace(invalid_bool.find("wheel_valid=true"), 16,
                         "wheel_valid=maybe");
    auto bad_bool = codec.parse_lines({invalid_bool});
    expect(bad_bool.records.front().message == "parse_error_invalid_bool_wheel_valid",
           "invalid bool detected");

    auto empty_ranges = line;
    const auto ranges_start = empty_ranges.find("tof_ranges=1.0;1.2;nan;1.5");
    const auto ranges_end = empty_ranges.find(",tof_angle_min=", ranges_start);
    empty_ranges.replace(ranges_start, ranges_end - ranges_start, "tof_ranges=");
    auto bad_ranges = codec.parse_lines({empty_ranges});
    expect(bad_ranges.records.front().message == "parse_error_empty_tof_ranges",
           "empty ranges detected");

    auto nan_log = codec.parse_lines({line});
    expect(nan_log.records.front().kind == RealSensorReplayRecordKind::Packet,
           "nan range packet accepted");

    auto comment = codec.parse_lines({"# comment"});
    expect(comment.records.front().kind == RealSensorReplayRecordKind::Comment,
           "comment remains comment");
    expect(invalid.records.front().kind != RealSensorReplayRecordKind::Comment,
           "parse error is not comment");

    const auto written = codec.write_text(nan_log);
    expect(written.find("tof_req_start=") != std::string::npos,
           "writes tof request start");
    expect(written.find("wheel_resp=") != std::string::npos,
           "writes wheel response");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
