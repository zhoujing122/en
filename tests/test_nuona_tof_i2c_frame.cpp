#include "robot_slamd/sensors/nuona_tof_i2c_frame.hpp"

#include <iostream>
#include <stdexcept>

int main() {
    using namespace robot_slamd;
    const auto valid = parse_nuona_tof_i2c_frame(encode_nuona_tof_i2c_frame(20, 0));
    if (!valid.valid || valid.distance_mm != 20 || valid.confidence != 0) return 1;
    const auto max = parse_nuona_tof_i2c_frame(encode_nuona_tof_i2c_frame(4500, 100));
    if (!max.valid || max.distance_mm != 4500 || max.confidence != 100) return 1;
    auto malformed = encode_nuona_tof_i2c_frame(100, 70);
    for (const auto &case_item : {std::pair<int, TofFrameError>{0, TofFrameError::InvalidHeader},
                                  {6, TofFrameError::InvalidDelimiter},
                                  {10, TofFrameError::InvalidTrailer}}) {
        auto bytes = malformed;
        bytes[static_cast<std::size_t>(case_item.first)] = 'X';
        if (parse_nuona_tof_i2c_frame(bytes).error != case_item.second) return 1;
    }
    malformed[1] = 'x';
    if (parse_nuona_tof_i2c_frame(malformed).error != TofFrameError::InvalidDistanceDigits) return 1;
    malformed = encode_nuona_tof_i2c_frame(100, 70);
    malformed[7] = 'x';
    if (parse_nuona_tof_i2c_frame(malformed).error != TofFrameError::InvalidConfidenceDigits) return 1;
    if (parse_nuona_tof_i2c_frame(encode_nuona_tof_i2c_frame(19, 70)).error != TofFrameError::DistanceOutOfRange) return 1;
    if (parse_nuona_tof_i2c_frame(encode_nuona_tof_i2c_frame(100, 101)).error != TofFrameError::ConfidenceOutOfRange) return 1;
    std::vector<std::uint8_t> short_frame(10);
    if (parse_nuona_tof_i2c_frame(short_frame).error != TofFrameError::InvalidLength) return 1;
    const auto timed = make_timed_tof_frame(valid, 100, 140);
    if (timed.sample_timestamp_us != 120) return 1;
    return 0;
}
