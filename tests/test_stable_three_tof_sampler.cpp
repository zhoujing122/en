#include "robot_slamd/sensors/stable_three_tof_sampler.hpp"

#include <array>
#include <cstdint>

int main() {
    using namespace robot_slamd;
    int call = 0;
    auto reader = [&call]() {
        const std::uint16_t values[] = {100, 102, 98, 101, 99};
        const auto frame = parse_nuona_tof_i2c_frame(encode_nuona_tof_i2c_frame(values[call++ % 5], 90));
        return make_timed_tof_frame(frame, 1000 + call * 10, 1004 + call * 10);
    };
    StableThreeTofSampler sampler({reader, reader, reader});
    const auto sample = sampler.acquire();
    if (!sample.usable_for_mapping || sample.front.distance_mm != 100 ||
        sample.front.confidence != 90 || sample.front.valid_sample_count != 5 ||
        sample.front.sample_timestamp_us == 0) return 1;
    int invalid_calls = 0;
    auto invalid_reader = [&invalid_calls]() {
        ++invalid_calls;
        TofI2cFrame invalid;
        invalid.error = TofFrameError::InvalidHeader;
        return make_timed_tof_frame(invalid, 2000 + invalid_calls, 2001 + invalid_calls);
    };
    StableThreeTofSampler fewer({invalid_reader, reader, reader});
    const auto rejected = fewer.acquire();
    if (rejected.front.valid || rejected.front.usable_for_mapping ||
        rejected.usable_for_mapping || rejected.front.valid_sample_count != 0) return 1;
    return 0;
}
