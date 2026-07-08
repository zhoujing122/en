#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <string>

namespace robot_slamd {

class RobotSlamMapPort {
public:
    virtual ~RobotSlamMapPort() = default;

    virtual bool ready() const = 0;

    virtual bool integrate_sensor_snapshot(const RobotSlamSensorSnapshot &snapshot,
                                           double now_s) = 0;

    virtual RobotSlamMapQuality latest_quality(double now_s) const = 0;

    virtual bool save_map(const std::string &output_path) = 0;

    virtual std::string status() const = 0;
};

class FakeRobotSlamMapPort final : public RobotSlamMapPort {
public:
    explicit FakeRobotSlamMapPort(int scans_to_good = 5)
        : scans_to_good_(scans_to_good) {}

    bool ready() const override {
        return ready_;
    }

    bool integrate_sensor_snapshot(const RobotSlamSensorSnapshot &snapshot,
                                   double now_s) override {
        (void)now_s;
        if (!ready_ || !snapshot.has_tof) {
            return false;
        }
        integrated_scans_++;
        return true;
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        (void)now_s;
        RobotSlamMapQuality quality;
        quality.good_enough = integrated_scans_ >= scans_to_good_;
        quality.coverage_ratio = scans_to_good_ <= 0
                                     ? 1.0
                                     : static_cast<double>(integrated_scans_) /
                                           static_cast<double>(scans_to_good_);
        if (quality.coverage_ratio > 1.0) {
            quality.coverage_ratio = 1.0;
        }
        quality.yaw_coverage_ratio = quality.coverage_ratio;
        quality.valid_scan_count = integrated_scans_;
        quality.reason = quality.good_enough ? "map_quality_good" : "map_quality_poor";
        return quality;
    }

    bool save_map(const std::string &output_path) override {
        saved_map_path = output_path;
        save_count++;
        return ready_;
    }

    std::string status() const override {
        return ready_ ? "fake_map_ready" : "fake_map_not_ready";
    }

    void set_ready(bool ready) {
        ready_ = ready;
    }

    int integrated_scans() const {
        return integrated_scans_;
    }

    std::string saved_map_path;
    int save_count = 0;

private:
    int scans_to_good_ = 5;
    int integrated_scans_ = 0;
    bool ready_ = true;
};

} // namespace robot_slamd
