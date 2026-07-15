#include "robot_slamd/localization/sparse_tof/localization_health_tracker.hpp"

#include <stdexcept>

namespace {
void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

robot_slamd::LocalizationHealthSample sample(
    double timestamp_s,
    robot_slamd::SparseTofLocalMatchStatus status,
    double odom_x_m = 0.0) {
    robot_slamd::LocalizationHealthSample out;
    out.timestamp_s = timestamp_s;
    out.match_status = status;
    out.odom_pose.x_m = odom_x_m;
    return out;
}
}

int main() {
    using namespace robot_slamd;
    LocalizationHealthConfig config;
    config.max_consecutive_consistency_failures = 3;
    config.minimum_lost_duration_s = 1.0;
    config.minimum_lost_odom_distance_m = 0.20;

    LocalizationHealthTracker ambiguity(config);
    ambiguity.update(sample(
        0.0, SparseTofLocalMatchStatus::AcceptedYawOnly));
    for (int i = 1; i <= 8; ++i) {
        const auto update = ambiguity.update(sample(
            static_cast<double>(i),
            i % 2 == 0
                ? SparseTofLocalMatchStatus::RejectedLowMargin
                : SparseTofLocalMatchStatus::RejectedMultimodal,
            i * 0.20));
        expect(update.state == LocalizationHealthState::Degraded &&
                   !update.became_lost,
               "ambiguity rejects degrade but never declare lost");
    }

    LocalizationHealthTracker mismatch(config);
    mismatch.update(sample(
        0.0, SparseTofLocalMatchStatus::AcceptedYawOnly));
    expect(mismatch.update(sample(
               0.2, SparseTofLocalMatchStatus::RejectedLowScore, 0.05)).state ==
               LocalizationHealthState::Degraded,
           "first consistency failure degrades");
    expect(mismatch.update(sample(
               0.5, SparseTofLocalMatchStatus::RejectedContradictory,
               0.10)).state == LocalizationHealthState::Degraded,
           "second consistency failure remains degraded");
    const auto lost = mismatch.update(sample(
        1.1, SparseTofLocalMatchStatus::RejectedNoValidCandidate, 0.15));
    expect(lost.state == LocalizationHealthState::Lost &&
               lost.became_lost,
           "bounded consecutive consistency failures plus time trigger lost");

    mismatch.mark_recovered(2.0, RobotPose2D{0.15, 0.0, 0.0});
    expect(mismatch.state() == LocalizationHealthState::Healthy,
           "confirmed recovery restores healthy");
    expect(mismatch.update(sample(
               2.1, SparseTofLocalMatchStatus::RejectedFlatCurve,
               0.15)).state == LocalizationHealthState::Degraded,
           "post-recovery ambiguity still does not become lost");

    LocalizationHealthTracker sensor_fault(config);
    auto invalid = sample(0.0, SparseTofLocalMatchStatus::NotRun);
    invalid.wheel_fresh = false;
    expect(sensor_fault.update(invalid).state ==
               LocalizationHealthState::Lost,
           "severe canonical sensor fault can immediately declare lost");
    return 0;
}
