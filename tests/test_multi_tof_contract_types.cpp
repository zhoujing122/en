#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_types.hpp"

#include <cmath>
#include <iostream>
#include <string>

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
    MultiTofContractOptions options;
    expect(to_string(MultiTofMountId::Front) == "front", "front string");
    expect(to_string(MultiTofContractStatus::Warning) == "warning", "status string");
    expect(to_string(MultiTofContractFault::DuplicateFrameId) == "duplicate_frame_id", "fault string");
    expect(multi_tof_mount_id_id(MultiTofMountId::Left) == 1, "mount id stable");
    expect(multi_tof_contract_status_id(MultiTofContractStatus::Fault) == 3, "status id stable");
    expect(multi_tof_contract_fault_id(MultiTofContractFault::InvalidPacketTimestamp) > 0, "fault id stable");
    expect(std::fabs(options.front_mount_yaw_rad - 0.0) < 1e-12, "front yaw default");
    expect(std::fabs(options.left_mount_yaw_rad - 1.5707963267948966) < 1e-12, "left yaw default");
    expect(std::fabs(options.right_mount_yaw_rad + 1.5707963267948966) < 1e-12, "right yaw default");
    expect(options.min_required_tof_count == 3, "min count default");
    return failures ? 1 : 0;
}
