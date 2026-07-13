#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_slamd {

struct ScalarTof9BytePayload {
    uint64_t echo_tag_u48 = 0;
    uint16_t distance_mm = 0;
    uint8_t confidence = 0;
};

enum class ScalarTofDecodeFault {
    None,
    InvalidLength,
    InvalidHex
};

struct ScalarTofDecodeResult {
    bool ok = false;
    ScalarTof9BytePayload payload;
    ScalarTofDecodeFault fault = ScalarTofDecodeFault::None;
    std::string reason;
};

inline uint64_t scalar_tof_decode_le_u48(const uint8_t *data) {
    uint64_t value = 0;
    for (int i = 0; i < 6; ++i) {
        value |= static_cast<uint64_t>(data[i]) << (8 * i);
    }
    return value;
}

inline uint16_t scalar_tof_decode_le_u16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

inline ScalarTofDecodeResult decode_scalar_tof_9byte_payload(
    const std::array<uint8_t, 9> &bytes) {
    ScalarTofDecodeResult result;
    result.ok = true;
    result.payload.echo_tag_u48 = scalar_tof_decode_le_u48(bytes.data());
    result.payload.distance_mm = scalar_tof_decode_le_u16(bytes.data() + 6);
    result.payload.confidence = bytes[8];
    result.reason = "ok";
    return result;
}

inline ScalarTofDecodeResult decode_scalar_tof_9byte_payload(
    const std::vector<uint8_t> &bytes) {
    if (bytes.size() != 9) {
        ScalarTofDecodeResult result;
        result.ok = false;
        result.fault = ScalarTofDecodeFault::InvalidLength;
        result.reason = "invalid_length";
        return result;
    }
    std::array<uint8_t, 9> fixed{};
    for (std::size_t i = 0; i < fixed.size(); ++i) {
        fixed[i] = bytes[i];
    }
    return decode_scalar_tof_9byte_payload(fixed);
}

inline std::array<uint8_t, 9> encode_scalar_tof_9byte_payload(
    const ScalarTof9BytePayload &payload) {
    std::array<uint8_t, 9> bytes{};
    const uint64_t echo = payload.echo_tag_u48 & 0x0000ffffffffffffULL;
    for (int i = 0; i < 6; ++i) {
        bytes[static_cast<std::size_t>(i)] =
            static_cast<uint8_t>((echo >> (8 * i)) & 0xffU);
    }
    bytes[6] = static_cast<uint8_t>(payload.distance_mm & 0xffU);
    bytes[7] = static_cast<uint8_t>((payload.distance_mm >> 8) & 0xffU);
    bytes[8] = payload.confidence;
    return bytes;
}

inline int scalar_tof_hex_nibble(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }
    return -1;
}

inline ScalarTofDecodeResult decode_scalar_tof_9byte_hex(
    const std::string &hex) {
    if (hex.size() != 18) {
        ScalarTofDecodeResult result;
        result.ok = false;
        result.fault = ScalarTofDecodeFault::InvalidLength;
        result.reason = "invalid_hex_length";
        return result;
    }
    std::array<uint8_t, 9> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        const int hi = scalar_tof_hex_nibble(hex[2 * i]);
        const int lo = scalar_tof_hex_nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) {
            ScalarTofDecodeResult result;
            result.ok = false;
            result.fault = ScalarTofDecodeFault::InvalidHex;
            result.reason = "invalid_hex_digit";
            return result;
        }
        bytes[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return decode_scalar_tof_9byte_payload(bytes);
}

} // namespace robot_slamd
