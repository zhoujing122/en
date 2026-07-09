#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_mount_config.hpp"

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
    MultiTofContractOptions options;
    auto config = MultiTofMountConfigFactory::default_three_tof_0_plus90_minus90();
    expect(config.front.frame_id == "tof_front_frame", "front frame");
    expect(config.left.frame_id == "tof_left_frame", "left frame");
    expect(config.right.frame_id == "tof_right_frame", "right frame");
    expect(validate_mount_config(config, options).ok, "default config ok");
    auto invalid = config;
    invalid.right.frame_id = invalid.left.frame_id;
    expect(!validate_mount_config(invalid, options).ok, "duplicate frame id fail");
    invalid = config;
    invalid.right.mount_id = MultiTofMountId::Left;
    expect(!validate_mount_config(invalid, options).ok, "duplicate mount id fail");
    invalid = config;
    invalid.left.yaw_rad = 0.0;
    expect(!validate_mount_config(invalid, options).ok, "wrong yaw fail");
    return failures ? 1 : 0;
}
