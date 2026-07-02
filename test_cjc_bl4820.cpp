#include "config.hpp"
#include "sensors.hpp"

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

    std::vector<uint8_t> resp{0xff, 0xff, 0x01, 0x0b, 0x00, 0x04, 0x02, 0x24,
                              0xa6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23};
    CjcBl4820WheelReading reading;
    std::string reason;
    bool checksum_error = false;
    expect(cjc_bl4820::parse_read_response(resp, 0x01, reading, reason, checksum_error), "valid read response should parse");
    expect(reading.position_raw == 166, "parsed position should be 166");
    expect(reading.rpm == 0, "parsed rpm should be 0");
    expect(reading.current_raw == 0, "parsed current should be 0");

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
    expect_no_throw([&] { validate_config(valid); }, "valid dual UART BL4820 config should pass");

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
