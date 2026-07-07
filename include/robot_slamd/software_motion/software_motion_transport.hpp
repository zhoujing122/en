#pragma once
#include "robot_slamd/software_motion/software_motion_command.hpp"

namespace robot_slamd {

struct SoftwareMotionSendResult {
    bool ok = false;
    bool accepted = false;
    std::string error;
    uint64_t transport_sequence = 0;
    double timestamp_s = 0.0;
};

class SoftwareMotionCommandTransport {
public:
    virtual ~SoftwareMotionCommandTransport() = default;

    virtual SoftwareMotionSendResult send_command(const SoftwareMotionCommand &command) = 0;
};

} // namespace robot_slamd
