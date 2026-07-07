#pragma once
#include "robot_slamd/core/common.hpp"

namespace robot_slamd {

struct MotionWriteResult {
    bool ok = false;
    std::string error;
};

class MotionCommandWriter {
public:
    virtual ~MotionCommandWriter() = default;

    virtual MotionWriteResult write_zero(double timestamp_s) = 0;

    virtual MotionWriteResult write_wheel_rpm(double timestamp_s,
                                              double left_rpm,
                                              double right_rpm) = 0;
};

class NullMotionCommandWriter final : public MotionCommandWriter {
public:
    MotionWriteResult write_zero(double) override {
        return {true, ""};
    }

    MotionWriteResult write_wheel_rpm(double, double, double) override {
        return {true, ""};
    }
};

class FakeMotionCommandWriter final : public MotionCommandWriter {
public:
    MotionWriteResult write_zero(double timestamp_s) override {
        if (fail_next_write) {
            fail_next_write = false;
            return {false, fail_reason};
        }
        zero_write_count++;
        last_timestamp_s = timestamp_s;
        last_left_rpm = 0.0;
        last_right_rpm = 0.0;
        return {true, ""};
    }

    MotionWriteResult write_wheel_rpm(double timestamp_s,
                                      double left_rpm,
                                      double right_rpm) override {
        if (fail_next_write) {
            fail_next_write = false;
            return {false, fail_reason};
        }
        rpm_write_count++;
        last_timestamp_s = timestamp_s;
        last_left_rpm = left_rpm;
        last_right_rpm = right_rpm;
        return {true, ""};
    }

    uint64_t zero_write_count = 0;
    uint64_t rpm_write_count = 0;
    double last_timestamp_s = 0.0;
    double last_left_rpm = 0.0;
    double last_right_rpm = 0.0;
    bool fail_next_write = false;
    std::string fail_reason = "fake_write_failure";
};

} // namespace robot_slamd
