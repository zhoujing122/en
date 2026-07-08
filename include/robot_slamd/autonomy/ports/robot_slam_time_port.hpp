#pragma once

namespace robot_slamd {

class RobotSlamTimePort {
public:
    virtual ~RobotSlamTimePort() = default;

    virtual double now_s() const = 0;
};

class FixedStepTimePort final : public RobotSlamTimePort {
public:
    explicit FixedStepTimePort(double start_s = 100.0, double step_s = 0.1)
        : now_s_(start_s), step_s_(step_s) {}

    double now_s() const override {
        return now_s_;
    }

    void advance() {
        now_s_ += step_s_;
    }

private:
    double now_s_ = 100.0;
    double step_s_ = 0.1;
};

} // namespace robot_slamd
