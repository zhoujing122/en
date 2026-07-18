#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_slamd {

constexpr std::size_t kNuonaTofI2cFrameSize = 11;
constexpr std::uint8_t kNuonaTofI2cFrameHeader = 0x20;
constexpr std::uint8_t kNuonaTofI2cFrameDelimiter = 0x2c;
constexpr std::uint8_t kNuonaTofI2cFrameTrailer = 0x0a;
constexpr std::uint16_t kNuonaTofProtocolMinDistanceMm = 20;
constexpr std::uint16_t kNuonaTofProtocolMaxDistanceMm = 4500;

enum class TofFrameError {
    None,
    InvalidLength,
    InvalidHeader,
    InvalidDelimiter,
    InvalidTrailer,
    InvalidDistanceDigits,
    InvalidConfidenceDigits,
    DistanceOutOfRange,
    ConfidenceOutOfRange
};

struct TofI2cFrame {
    bool valid = false;
    std::uint16_t distance_mm = 0;
    std::uint8_t confidence = 0;
    TofFrameError error = TofFrameError::InvalidLength;
    std::string reason = "invalid";
};

struct TimedTofFrame {
    TofI2cFrame frame;
    std::uint64_t request_start_us = 0;
    std::uint64_t response_received_us = 0;
    std::uint64_t sample_timestamp_us = 0;
};

inline const char *to_string(TofFrameError error) {
    switch (error) {
    case TofFrameError::None: return "none";
    case TofFrameError::InvalidLength: return "invalid_length";
    case TofFrameError::InvalidHeader: return "invalid_header";
    case TofFrameError::InvalidDelimiter: return "invalid_delimiter";
    case TofFrameError::InvalidTrailer: return "invalid_trailer";
    case TofFrameError::InvalidDistanceDigits: return "invalid_distance_digits";
    case TofFrameError::InvalidConfidenceDigits: return "invalid_confidence_digits";
    case TofFrameError::DistanceOutOfRange: return "distance_out_of_range";
    case TofFrameError::ConfidenceOutOfRange: return "confidence_out_of_range";
    }
    return "unknown";
}

inline TofI2cFrame tof_frame_error(TofFrameError error) {
    TofI2cFrame result;
    result.error = error;
    result.reason = to_string(error);
    return result;
}

inline TofI2cFrame parse_nuona_tof_i2c_frame(const std::uint8_t *bytes,
                                             std::size_t length) {
    if (!bytes || length != kNuonaTofI2cFrameSize) {
        return tof_frame_error(TofFrameError::InvalidLength);
    }
    if (bytes[0] != kNuonaTofI2cFrameHeader) {
        return tof_frame_error(TofFrameError::InvalidHeader);
    }
    if (bytes[6] != kNuonaTofI2cFrameDelimiter) {
        return tof_frame_error(TofFrameError::InvalidDelimiter);
    }
    if (bytes[10] != kNuonaTofI2cFrameTrailer) {
        return tof_frame_error(TofFrameError::InvalidTrailer);
    }

    unsigned distance = 0;
    for (std::size_t i = 1; i <= 5; ++i) {
        if (bytes[i] < '0' || bytes[i] > '9') {
            return tof_frame_error(TofFrameError::InvalidDistanceDigits);
        }
        distance = distance * 10U + static_cast<unsigned>(bytes[i] - '0');
    }
    unsigned confidence = 0;
    for (std::size_t i = 7; i <= 9; ++i) {
        if (bytes[i] < '0' || bytes[i] > '9') {
            return tof_frame_error(TofFrameError::InvalidConfidenceDigits);
        }
        confidence = confidence * 10U + static_cast<unsigned>(bytes[i] - '0');
    }
    if (distance < kNuonaTofProtocolMinDistanceMm ||
        distance > kNuonaTofProtocolMaxDistanceMm) {
        return tof_frame_error(TofFrameError::DistanceOutOfRange);
    }
    if (confidence > 100U) {
        return tof_frame_error(TofFrameError::ConfidenceOutOfRange);
    }

    TofI2cFrame result;
    result.valid = true;
    result.distance_mm = static_cast<std::uint16_t>(distance);
    result.confidence = static_cast<std::uint8_t>(confidence);
    result.error = TofFrameError::None;
    result.reason = "valid_distance";
    return result;
}

inline TofI2cFrame parse_nuona_tof_i2c_frame(
    const std::array<std::uint8_t, kNuonaTofI2cFrameSize> &bytes) {
    return parse_nuona_tof_i2c_frame(bytes.data(), bytes.size());
}

inline TofI2cFrame parse_nuona_tof_i2c_frame(
    const std::vector<std::uint8_t> &bytes) {
    return parse_nuona_tof_i2c_frame(bytes.data(), bytes.size());
}

inline TimedTofFrame make_timed_tof_frame(
    const TofI2cFrame &frame,
    std::uint64_t request_start_us,
    std::uint64_t response_received_us) {
    TimedTofFrame result;
    result.frame = frame;
    result.request_start_us = request_start_us;
    result.response_received_us = response_received_us;
    result.sample_timestamp_us = request_start_us +
        (response_received_us >= request_start_us
             ? (response_received_us - request_start_us) / 2U
             : 0U);
    return result;
}

inline std::array<std::uint8_t, kNuonaTofI2cFrameSize>
encode_nuona_tof_i2c_frame(std::uint16_t distance_mm,
                           std::uint8_t confidence) {
    std::array<std::uint8_t, kNuonaTofI2cFrameSize> bytes{};
    bytes[0] = kNuonaTofI2cFrameHeader;
    bytes[6] = kNuonaTofI2cFrameDelimiter;
    bytes[10] = kNuonaTofI2cFrameTrailer;
    for (int index = 5; index >= 1; --index) {
        bytes[static_cast<std::size_t>(index)] =
            static_cast<std::uint8_t>('0' + distance_mm % 10U);
        distance_mm = static_cast<std::uint16_t>(distance_mm / 10U);
    }
    for (int index = 9; index >= 7; --index) {
        bytes[static_cast<std::size_t>(index)] =
            static_cast<std::uint8_t>('0' + confidence % 10U);
        confidence = static_cast<std::uint8_t>(confidence / 10U);
    }
    return bytes;
}

} // namespace robot_slamd
