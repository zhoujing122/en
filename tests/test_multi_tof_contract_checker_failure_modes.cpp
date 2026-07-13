#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sample_data.hpp"

#include <iostream>

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
    MultiTofContractChecker checker;
    expect(!checker.check_packet(MultiTofSampleData::missing_left_packet(), 100.0).ok, "missing left fail");
    expect(!checker.check_packet(MultiTofSampleData::duplicate_frame_id_packet(), 100.0).ok, "duplicate frame fail");
    expect(!checker.check_packet(MultiTofSampleData::duplicate_mount_id_packet(), 100.0).ok, "duplicate mount fail");
    expect(!checker.check_packet(MultiTofSampleData::invalid_mount_yaw_packet(), 100.0).ok, "invalid yaw fail");
    expect(!checker.check_packet(MultiTofSampleData::invalid_distance_packet(), 100.0).ok, "invalid distance fail");
    expect(!checker.check_packet(MultiTofSampleData::latency_too_high_packet(), 100.0).ok, "latency too high fail");
    auto bad_timing = MultiTofSampleData::valid_three_tof_packet();
    bad_timing.front.timing.request_latency_s = 0.001;
    expect(!checker.check_packet(bad_timing, 100.0).ok, "bad request timing fail");
    MultiTofContractOptions relaxed;
    relaxed.require_left = false;
    relaxed.min_required_tof_count = 2;
    MultiTofContractChecker relaxed_checker(relaxed);
    expect(relaxed_checker.check_packet(MultiTofSampleData::missing_left_packet(), 100.0).ok,
           "missing left can pass with min two and left not required");
    return failures ? 1 : 0;
}
