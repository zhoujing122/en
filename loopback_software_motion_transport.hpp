#pragma once
// M2-B1-pre shadow/loopback only, no hardware IO.
#include "software_motion_transport.hpp"

namespace robot_slamd {

class LoopbackSoftwareMotionCommandTransport final : public SoftwareMotionCommandTransport {
public:
    explicit LoopbackSoftwareMotionCommandTransport(bool shadow = true)
        : shadow_mode(shadow) {}

    SoftwareMotionSendResult send_command(const SoftwareMotionCommand &command) override {
        send_count++;
        if (require_shadow_mode && !shadow_mode) {
            reject_count++;
            return {true, false, "loopback_shadow_mode_required", send_count, command.timestamp_s};
        }
        if (reject_non_stop &&
            command.direction != SoftwareMotionDirection::Stop &&
            command.direction != SoftwareMotionDirection::EmergencyStop) {
            reject_count++;
            return {true, false, "loopback_non_stop_rejected", send_count, command.timestamp_s};
        }
        sent_commands.push_back(command);
        return {true, true, "", send_count, command.timestamp_s};
    }

    void clear() {
        sent_commands.clear();
        send_count = 0;
        reject_count = 0;
    }

    bool shadow_mode = true;
    bool require_shadow_mode = true;
    bool reject_non_stop = false;
    std::vector<SoftwareMotionCommand> sent_commands;
    uint64_t send_count = 0;
    uint64_t reject_count = 0;
};

} // namespace robot_slamd
