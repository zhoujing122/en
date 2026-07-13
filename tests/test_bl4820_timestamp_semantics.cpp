#include "robot_slamd/sensors/sensors.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    expect(cjc_bl4820::read_midpoint_us(1000000, 1001200) == 1000600,
           "read midpoint");
    expect(cjc_bl4820::pair_midpoint_us(1000600, 1002600) == 1001600,
           "pair midpoint");
    expect(cjc_bl4820::pair_skew_us(static_cast<uint64_t>(1000600), static_cast<uint64_t>(1002600)) == 2000,
           "pair skew");

    CjcBl4820WheelReading reading;
    std::string reason;
    const std::vector<uint8_t> payload{0xe8, 0x03, 0x9c, 0xff, 0x0a, 0x00};
    expect(cjc_bl4820::parse_read_payload(payload, reading, reason), "payload parses");
    expect(reading.position_raw == 1000, "position 1000");
    expect(reading.rpm == -100, "rpm -100");
    expect(reading.current_raw == 10, "current raw");
    expect(std::fabs(reading.current_a - 1.0) < 1e-12, "current amps");
    return failures ? 1 : 0;
}
