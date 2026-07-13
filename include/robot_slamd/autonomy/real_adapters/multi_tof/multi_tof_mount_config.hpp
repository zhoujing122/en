#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_types.hpp"

#include <set>

namespace robot_slamd {

struct MultiTofMountFrameConfig {
    MultiTofMountId mount_id = MultiTofMountId::Front;
    std::string frame_id;
    double yaw_rad = 0.0;
    double fov_min_rad = 0.0;
    double fov_max_rad = 0.0;
};

struct MultiTofMountConfig {
    MultiTofMountFrameConfig front;
    MultiTofMountFrameConfig left;
    MultiTofMountFrameConfig right;
};

class MultiTofMountConfigFactory {
public:
    static MultiTofMountConfig default_three_tof_0_plus90_minus90() {
        MultiTofMountConfig config;
        config.front.mount_id = MultiTofMountId::Front;
        config.front.frame_id = "tof_front_frame";
        config.front.yaw_rad = 0.0;
        config.front.fov_min_rad = -0.5 * scalar_tof_default_full_fov_rad();
        config.front.fov_max_rad = 0.5 * scalar_tof_default_full_fov_rad();
        config.left.mount_id = MultiTofMountId::Left;
        config.left.frame_id = "tof_left_frame";
        config.left.yaw_rad = 1.5707963267948966;
        config.left.fov_min_rad = -0.5 * scalar_tof_default_full_fov_rad();
        config.left.fov_max_rad = 0.5 * scalar_tof_default_full_fov_rad();
        config.right.mount_id = MultiTofMountId::Right;
        config.right.frame_id = "tof_right_frame";
        config.right.yaw_rad = -1.5707963267948966;
        config.right.fov_min_rad = -0.5 * scalar_tof_default_full_fov_rad();
        config.right.fov_max_rad = 0.5 * scalar_tof_default_full_fov_rad();
        return config;
    }
};

inline MultiTofContractResult validate_mount_config(
    const MultiTofMountConfig &config,
    const MultiTofContractOptions &options) {
    MultiTofContractResult result;
    const MultiTofMountFrameConfig frames[3] = {
        config.front,
        config.left,
        config.right,
    };
    std::set<int> mount_ids;
    std::set<std::string> frame_ids;
    for (const auto &frame : frames) {
        const int mount_id = multi_tof_mount_id_id(frame.mount_id);
        if (!mount_ids.insert(mount_id).second) {
            result.failed.push_back("multi_tof_mount_duplicate_mount_id");
            result.fault = MultiTofContractFault::DuplicateMountId;
            result.message = "multi_tof_mount_duplicate_mount_id";
            return result;
        }
        if (frame.frame_id.empty()) {
            result.failed.push_back("multi_tof_mount_empty_frame_id");
            result.fault = MultiTofContractFault::EmptyFrameId;
            result.message = "multi_tof_mount_empty_frame_id";
            return result;
        }
        if (!frame_ids.insert(frame.frame_id).second) {
            result.failed.push_back("multi_tof_mount_duplicate_frame_id");
            result.fault = MultiTofContractFault::DuplicateFrameId;
            result.message = "multi_tof_mount_duplicate_frame_id";
            return result;
        }
        if (std::fabs(frame.yaw_rad - expected_mount_yaw_rad(frame.mount_id, options)) >
            options.max_mount_yaw_error_rad) {
            result.failed.push_back("multi_tof_mount_invalid_yaw");
            result.fault = MultiTofContractFault::InvalidMountYaw;
            result.message = "multi_tof_mount_invalid_yaw";
            return result;
        }
        if (frame.fov_min_rad > frame.fov_max_rad) {
            result.failed.push_back("multi_tof_mount_invalid_fov");
            result.fault = MultiTofContractFault::InvalidAngle;
            result.message = "multi_tof_mount_invalid_fov";
            return result;
        }
    }
    result.ok = true;
    result.status = MultiTofContractStatus::Accepted;
    result.fault = MultiTofContractFault::None;
    result.valid_tof_count = 3;
    result.passed.push_back("multi_tof_mount_config_ok");
    result.message = "multi_tof_mount_config_ok";
    return result;
}

} // namespace robot_slamd
