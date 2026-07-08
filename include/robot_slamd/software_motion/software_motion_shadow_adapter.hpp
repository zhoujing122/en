#pragma once
#include "robot_slamd/software_motion/software_motion_transport_contract_checker.hpp"

#include <memory>
#include <utility>

namespace robot_slamd {

struct SoftwareMotionShadowAdapterResult {
    bool ok = false;
    bool accepted = false;
    std::string error;
    SoftwareMotionCommand command;
    SoftwareMotionSendResult send_result;
};

class SoftwareMotionShadowAdapter final : public SoftwareMotionCommandTransport {
public:
    SoftwareMotionShadowAdapter(std::shared_ptr<SoftwareMotionCommandTransport> inner,
                                SoftwareMotionTransportContractChecker checker)
        : inner_(std::move(inner)), checker_(checker) {}

    SoftwareMotionSendResult send_command(const SoftwareMotionCommand &command) override {
        SoftwareMotionShadowAdapterResult record;
        record.command = command;

        // Missing inner transport.
        if (!inner_) {
            record.error = "shadow_adapter_inner_missing";
            history_.push_back(record);
            return {false, false, record.error, 0, command.timestamp_s};
        }

        // Pre-send command contract check.
        auto command_check = checker_.check_command(command, command.timestamp_s);
        if (!command_check.ok) {
            record.ok = true;
            record.accepted = false;
            record.error = command_check.error;
            record.send_result = {true, false, command_check.error, 0, command.timestamp_s};
            history_.push_back(record);
            return record.send_result;
        }

        // Inner transport send.
        auto send_result = inner_->send_command(command);
        record.send_result = send_result;

        // Post-send result contract check.
        auto result_check = checker_.check_send_result(command,
                                                       send_result,
                                                       command.timestamp_s);
        if (!result_check.ok) {
            record.ok = true;
            record.accepted = false;
            record.error = result_check.error;
            record.send_result = {true,
                                  false,
                                  result_check.error,
                                  send_result.transport_sequence,
                                  send_result.timestamp_s};
            history_.push_back(record);
            return record.send_result;
        }

        // History recording.
        record.ok = send_result.ok;
        record.accepted = send_result.accepted;
        record.error = send_result.error;
        history_.push_back(record);

        // Success return.
        return send_result;
    }

    const std::vector<SoftwareMotionShadowAdapterResult> &history() const {
        return history_;
    }

    void clear_history() {
        history_.clear();
    }

private:
    std::shared_ptr<SoftwareMotionCommandTransport> inner_;
    SoftwareMotionTransportContractChecker checker_;
    std::vector<SoftwareMotionShadowAdapterResult> history_;
};

} // namespace robot_slamd
