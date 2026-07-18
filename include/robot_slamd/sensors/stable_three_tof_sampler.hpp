#pragma once

#include "robot_slamd/sensors/nuona_tof_i2c_frame.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

enum class StableTofDirection {
    Front = 0,
    Left = 1,
    Right = 2
};

struct StableThreeTofSamplerConfig {
    std::size_t samples_per_sensor = 5;
    std::size_t min_valid_samples = 3;
    std::uint16_t protocol_min_distance_mm = kNuonaTofProtocolMinDistanceMm;
    std::uint16_t protocol_max_distance_mm = kNuonaTofProtocolMaxDistanceMm;
    std::uint16_t mapping_min_distance_mm = 30;
    std::uint16_t mapping_max_distance_mm = kNuonaTofProtocolMaxDistanceMm;
    std::uint8_t mapping_min_confidence = 70;
};

struct StableTofReading {
    bool valid = false;
    bool usable_for_mapping = false;
    std::uint16_t distance_mm = 0;
    std::uint8_t confidence = 0;
    std::uint64_t sample_timestamp_us = 0;
    std::size_t valid_sample_count = 0;
    std::string rejection_reason = "not_sampled";
    std::array<TimedTofFrame, 5> raw_frames{};
    std::size_t raw_frame_count = 0;
};

struct StableThreeTofSample {
    StableTofReading front;
    StableTofReading left;
    StableTofReading right;
    std::uint64_t acquisition_start_us = 0;
    std::uint64_t acquisition_end_us = 0;
    bool usable_for_mapping = false;
};

using TofFrameReader = std::function<TimedTofFrame()>;

inline bool stable_three_tof_sampler_config_valid(
    const StableThreeTofSamplerConfig &config) {
    return config.samples_per_sensor == 5 &&
           config.min_valid_samples > 0 &&
           config.min_valid_samples <= config.samples_per_sensor &&
           config.protocol_min_distance_mm >= kNuonaTofProtocolMinDistanceMm &&
           config.protocol_min_distance_mm <= config.protocol_max_distance_mm &&
           config.protocol_max_distance_mm <= kNuonaTofProtocolMaxDistanceMm &&
           config.mapping_min_distance_mm >= config.protocol_min_distance_mm &&
           config.mapping_min_distance_mm <= config.mapping_max_distance_mm &&
           config.mapping_max_distance_mm <= config.protocol_max_distance_mm &&
           config.mapping_min_confidence <= 100;
}

class StableThreeTofSampler {
public:
    StableThreeTofSampler(
        std::array<TofFrameReader, 3> readers,
        StableThreeTofSamplerConfig config = {})
        : readers_(std::move(readers)), config_(config) {}

    bool ready() const {
        return stable_three_tof_sampler_config_valid(config_) &&
               static_cast<bool>(readers_[0]) &&
               static_cast<bool>(readers_[1]) &&
               static_cast<bool>(readers_[2]);
    }

    StableThreeTofSample acquire() {
        StableThreeTofSample result;
        if (!ready()) {
            result.front.rejection_reason = "sampler_not_ready";
            result.left.rejection_reason = "sampler_not_ready";
            result.right.rejection_reason = "sampler_not_ready";
            return result;
        }
        result.front = acquire_one(StableTofDirection::Front,
                                   readers_[0], result);
        result.left = acquire_one(StableTofDirection::Left,
                                  readers_[1], result);
        result.right = acquire_one(StableTofDirection::Right,
                                   readers_[2], result);
        result.usable_for_mapping = result.front.usable_for_mapping &&
                                     result.left.usable_for_mapping &&
                                     result.right.usable_for_mapping;
        return result;
    }

    const StableThreeTofSamplerConfig &config() const { return config_; }

private:
    StableTofReading acquire_one(StableTofDirection direction,
                                 const TofFrameReader &reader,
                                 StableThreeTofSample &sample) {
        (void)direction;
        StableTofReading result;
        std::vector<TimedTofFrame> valid_frames;
        valid_frames.reserve(config_.samples_per_sensor);
        for (std::size_t index = 0; index < config_.samples_per_sensor; ++index) {
            const TimedTofFrame frame = reader();
            if (result.raw_frame_count < result.raw_frames.size()) {
                result.raw_frames[result.raw_frame_count++] = frame;
            }
            if (sample.acquisition_start_us == 0 ||
                frame.request_start_us < sample.acquisition_start_us) {
                sample.acquisition_start_us = frame.request_start_us;
            }
            sample.acquisition_end_us = std::max(
                sample.acquisition_end_us, frame.response_received_us);
            if (frame.frame.valid && frame.frame.distance_mm >=
                    config_.protocol_min_distance_mm &&
                frame.frame.distance_mm <= config_.protocol_max_distance_mm) {
                result.valid_sample_count++;
                valid_frames.push_back(frame);
            }
        }
        if (result.valid_sample_count < config_.min_valid_samples) {
            result.rejection_reason = "fewer_than_min_valid_samples";
            return result;
        }

        std::sort(valid_frames.begin(), valid_frames.end(),
                  [](const TimedTofFrame &lhs, const TimedTofFrame &rhs) {
                      return lhs.frame.distance_mm < rhs.frame.distance_mm;
                  });
        const TimedTofFrame &selected =
            valid_frames[valid_frames.size() / 2U];
        result.valid = true;
        result.distance_mm = selected.frame.distance_mm;
        result.confidence = selected.frame.confidence;
        result.sample_timestamp_us = selected.sample_timestamp_us;
        if (selected.frame.distance_mm < config_.mapping_min_distance_mm ||
            selected.frame.distance_mm > config_.mapping_max_distance_mm) {
            result.rejection_reason = "distance_outside_mapping_range";
            return result;
        }
        if (selected.frame.confidence < config_.mapping_min_confidence) {
            result.rejection_reason = "confidence_below_mapping_threshold";
            return result;
        }
        result.usable_for_mapping = true;
        result.rejection_reason = "stable_median_selected";
        return result;
    }

    std::array<TofFrameReader, 3> readers_;
    StableThreeTofSamplerConfig config_;
};

} // namespace robot_slamd
