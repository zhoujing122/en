#include "robot_slamd/autonomy/real_adapters/multi_tof/scalar_tof_9byte_codec.hpp"

#include <array>
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
    const std::array<uint8_t, 9> golden{{0x11, 0x22, 0x33, 0x44, 0x00, 0x00, 0xab, 0x0a, 0x64}};
    const auto decoded = decode_scalar_tof_9byte_payload(golden);
    expect(decoded.ok, "golden decodes");
    expect(decoded.payload.echo_tag_u48 == 0x000044332211ULL, "echo little endian");
    expect(decoded.payload.echo_tag_u48 == 1144201745ULL, "echo decimal");
    expect(decoded.payload.distance_mm == 2731, "distance little endian");
    expect(decoded.payload.confidence == 100, "confidence 100");

    expect(!decode_scalar_tof_9byte_payload(std::vector<uint8_t>(8, 0)).ok, "8 bytes rejected");
    expect(!decode_scalar_tof_9byte_payload(std::vector<uint8_t>(10, 0)).ok, "10 bytes rejected");
    expect(!decode_scalar_tof_9byte_payload(std::vector<uint8_t>()).ok, "empty rejected");

    const std::array<uint8_t, 9> zeros{};
    const auto zero = decode_scalar_tof_9byte_payload(zeros);
    expect(zero.ok && zero.payload.echo_tag_u48 == 0 && zero.payload.distance_mm == 0 &&
               zero.payload.confidence == 0,
           "all zero payload parses");

    const std::array<uint8_t, 9> max_echo{{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x34, 0x12, 0xff}};
    const auto maxed = decode_scalar_tof_9byte_payload(max_echo);
    expect(maxed.payload.echo_tag_u48 == 0x0000ffffffffffffULL, "max 48 bit echo");
    expect(maxed.payload.distance_mm == 0x1234, "distance endian");
    expect(maxed.payload.confidence == 255, "confidence 255 parses");

    ScalarTof9BytePayload payload;
    payload.echo_tag_u48 = 0xffffffffffffffffULL;
    payload.distance_mm = 12000;
    payload.confidence = 70;
    const auto roundtrip = decode_scalar_tof_9byte_payload(encode_scalar_tof_9byte_payload(payload));
    expect(roundtrip.ok, "roundtrip decodes");
    expect(roundtrip.payload.echo_tag_u48 == 0x0000ffffffffffffULL, "roundtrip masks echo to 48 bit");
    expect(roundtrip.payload.distance_mm == 12000, "roundtrip distance");
    expect(roundtrip.payload.confidence == 70, "roundtrip confidence");

    expect(decode_scalar_tof_9byte_hex("112233440000AB0A64").ok, "hex golden decodes");
    expect(!decode_scalar_tof_9byte_hex("112233").ok, "short hex rejected");
    expect(!decode_scalar_tof_9byte_hex("112233440000AB0A6Z").ok, "bad hex rejected");
    return failures ? 1 : 0;
}
