#include "robot_slamd/config/config.hpp"
#include "robot_slamd/sensors/sensors.hpp"

#include <cmath>
#include <functional>
#include <iostream>

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
        std::cerr << "FAIL: expected throw: " << message << "\n";
        failures++;
    } catch (const std::exception &) {
    }
}

void expect_near(double a, double b, double eps, const char *message) {
    if (std::fabs(a - b) > eps) {
        std::cerr << "FAIL: " << message << " got=" << a << " expected=" << b << "\n";
        failures++;
    }
}

void expect_no_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
    } catch (const std::exception &e) {
        std::cerr << "FAIL: unexpected throw: " << message << ": " << e.what() << "\n";
        failures++;
    }
}

} // namespace

int main() {
    using namespace robot_slamd;

    std::vector<uint8_t> req = cjc_bl4820::read_request(0x01);
    std::vector<uint8_t> expected_req{0xff, 0xff, 0x01, 0x04, 0x02, 0x24, 0x06, 0xce};
    expect(req == expected_req, "read request checksum should match FF FF 01 04 02 24 06 CE");

    uint8_t pos_bytes[2] = {0xa6, 0x00};
    expect(cjc_bl4820::le_u16(pos_bytes) == 166, "A6 00 should parse as uint16 166");

    uint8_t rpm_bytes[2] = {0x9c, 0xff};
    expect(cjc_bl4820::le_i16(rpm_bytes) == -100, "9C FF should parse as int16 -100 RPM");

    std::vector<uint8_t> payload{0xe8, 0x03, 0x9c, 0xff, 0x0a, 0x00};
    CjcBl4820WheelReading payload_reading;
    std::string payload_reason;
    expect(cjc_bl4820::parse_read_payload(payload, payload_reading, payload_reason), "6-byte payload should parse");
    expect(payload_reading.position_raw == 1000, "payload position should be 1000");
    expect(payload_reading.rpm == -100, "payload speed should be -100 rpm");
    expect(payload_reading.current_raw == 10, "payload current raw should be 10");
    expect_near(payload_reading.current_a, 1.0, 1e-12, "payload current should be 1.0 A");

    uint8_t pos_rpm_bytes[2] = {0x64, 0x00};
    expect(cjc_bl4820::le_i16(pos_rpm_bytes) == 100, "64 00 should parse as +100 RPM");
    expect_near(cjc_bl4820::current_raw_to_a(25), 2.5, 1e-12, "current raw should scale by 0.1 A");

    std::vector<uint8_t> bad_payload{0x01, 0x02};
    expect(!cjc_bl4820::parse_read_payload(bad_payload, payload_reading, payload_reason), "short payload should fail");

    std::vector<uint8_t> resp{0xff, 0xff, 0x01, 0x0b, 0x00, 0x04, 0x02, 0x24,
                              0xa6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23};
    CjcBl4820WheelReading reading;
    std::string reason;
    bool checksum_error = false;
    expect(cjc_bl4820::parse_read_response(resp, 0x01, reading, reason, checksum_error), "valid read response should parse");
    expect(reading.position_raw == 166, "parsed position should be 166");
    expect(reading.rpm == 0, "parsed rpm should be 0");
    expect(reading.current_raw == 0, "parsed current should be 0");
    expect_near(reading.current_a, 0.0, 1e-12, "parsed current amps should be 0");

    std::vector<uint8_t> neg_rpm_resp = resp;
    neg_rpm_resp[10] = 0x9c;
    neg_rpm_resp[11] = 0xff;
    neg_rpm_resp.back() = cjc_bl4820::checksum(neg_rpm_resp, 2, neg_rpm_resp.size() - 1);
    expect(cjc_bl4820::parse_read_response(neg_rpm_resp, 0x01, reading, reason, checksum_error), "negative rpm response should parse");
    expect(reading.rpm == -100, "parsed rpm should be -100");

    std::vector<uint8_t> bad_checksum = resp;
    bad_checksum.back() ^= 0x01;
    expect(!cjc_bl4820::parse_read_response(bad_checksum, 0x01, reading, reason, checksum_error), "bad checksum should be rejected");
    expect(checksum_error, "bad checksum should set checksum_error");

    EncoderStats parse_stats;
    bool stats_checksum = false;
    std::string stats_reason;
    expect(!cjc_bl4820::parse_read_response(std::vector<uint8_t>{0xff}, 0x01, reading, stats_reason, stats_checksum), "short frame should fail parsing");
    cjc_bl4820::record_parse_failure(parse_stats, true, stats_reason, stats_checksum);
    expect(parse_stats.left_parse_error_count == 1, "short frame should increment left parse errors via helper");
    cjc_bl4820::record_timeout(parse_stats, false);
    expect(parse_stats.right_timeout && parse_stats.right_timeout_count == 1, "timeout should update right timeout stats");
    cjc_bl4820::record_parse_failure(parse_stats, true, "checksum_error", true);
    expect(parse_stats.left_checksum_error_count == 1, "checksum failure should increment side checksum count");
    cjc_bl4820::record_parse_failure(parse_stats, false, "status_error", false);
    expect(parse_stats.right_status_error_count == 1, "status failure should increment side status count");

    std::vector<uint8_t> bad_status = resp;
    bad_status[4] = 0x03;
    bad_status.back() = cjc_bl4820::checksum(bad_status, 2, bad_status.size() - 1);
    expect(!cjc_bl4820::parse_read_response(bad_status, 0x01, reading, reason, checksum_error), "nonzero status should be rejected");
    expect(reason == "status_error", "nonzero status should report status_error");

    expect_near(cjc_bl4820::pair_skew_us(10.000000, 10.005000), 5000.0, 1e-6, "pair skew should be microseconds");
    expect(cjc_bl4820::pair_skew_ok(1.0, 1.005, 10000.0), "5ms pair skew should pass 10ms threshold");
    expect(!cjc_bl4820::pair_skew_ok(1.0, 1.020, 10000.0), "20ms pair skew should fail 10ms threshold");

    expect(cjc_bl4820::unwrap_delta(1000, 20, 1024) == 44, "unwrap 1000 -> 20 should be +44");
    expect(cjc_bl4820::unwrap_delta(20, 1000, 1024) == -44, "unwrap 20 -> 1000 should be -44");

    Config missing_ports;
    missing_ports.encoder_source = "cjc_bl4820_uart";
    missing_ports.encoder_ticks_per_rev = 1024;
    expect_throw([&] { validate_config(missing_ports); }, "cjc_bl4820_uart without ports should be invalid");

    Config same_port;
    same_port.encoder_source = "cjc_bl4820_uart";
    same_port.encoder_ticks_per_rev = 1024;
    same_port.encoder_left_port = "/dev/ttyS1";
    same_port.encoder_right_port = "/dev/ttyS1";
    expect_throw([&] { validate_config(same_port); }, "same left/right port should be invalid");

    Config valid;
    valid.encoder_source = "cjc_bl4820_uart";
    valid.encoder_ticks_per_rev = 1024;
    valid.encoder_left_port = "/dev/ttyS1";
    valid.encoder_right_port = "/dev/ttyS2";
    valid.encoder_left_id = 1;
    valid.encoder_right_id = 1;
    valid.encoder_max_pair_skew_us = 10000.0;
    valid.encoder_max_read_latency_us = 10000.0;
    expect_no_throw([&] { validate_config(valid); }, "valid dual UART BL4820 config should pass");

    Config bad_skew_cfg = valid;
    bad_skew_cfg.encoder_max_pair_skew_us = -1.0;
    expect_throw([&] { validate_config(bad_skew_cfg); }, "negative pair skew threshold should be invalid");

    Config bad_latency_cfg = valid;
    bad_latency_cfg.encoder_max_read_latency_us = -1.0;
    expect_throw([&] { validate_config(bad_latency_cfg); }, "negative read latency threshold should be invalid");

    Config no_unwrap = valid;
    no_unwrap.encoder_position_unwrap = false;
    expect_throw([&] { validate_config(no_unwrap); }, "position_unwrap=false should be invalid for BL4820 odometry");

    if (failures) {
        std::cerr << failures << " test failures\n";
        return 1;
    }
    std::cout << "test_cjc_bl4820 ok\n";
    return 0;
}
