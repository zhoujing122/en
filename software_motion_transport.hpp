#pragma once
#include "software_motion_command.hpp"

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

class FakeSoftwareMotionCommandTransport final : public SoftwareMotionCommandTransport {
public:
    SoftwareMotionSendResult send_command(const SoftwareMotionCommand &command) override {
        send_count++;
        if (fail_next_send) {
            fail_next_send = false;
            return {false, false, fail_reason, send_count, command.timestamp_s};
        }
        if (reject_next_send) {
            reject_next_send = false;
            return {true, false, reject_reason, send_count, command.timestamp_s};
        }
        sent_commands.push_back(command);
        return {true, true, "", send_count, command.timestamp_s};
    }

    void clear() {
        sent_commands.clear();
        send_count = 0;
        fail_next_send = false;
        reject_next_send = false;
    }

    std::vector<SoftwareMotionCommand> sent_commands;
    uint64_t send_count = 0;
    bool fail_next_send = false;
    bool reject_next_send = false;
    std::string fail_reason = "fake_software_motion_send_failed";
    std::string reject_reason = "fake_software_motion_rejected";
};

class LoopbackSoftwareMotionCommandTransport final : public SoftwareMotionCommandTransport {
public:
    explicit LoopbackSoftwareMotionCommandTransport(bool shadow = true) : shadow_mode(shadow) {}

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
