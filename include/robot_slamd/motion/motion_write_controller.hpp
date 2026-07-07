#pragma once
#include "robot_slamd/motion/bl4820_motion_safety_executor.hpp"
#include "robot_slamd/motion/motion_command_writer.hpp"

namespace robot_slamd {

struct MotionWriterDispatchSnapshot {
    double timestamp_s = 0.0;
    bool dispatch_enabled = false;
    bool attempted = false;
    bool wrote_zero = false;
    bool wrote_rpm = false;
    bool writer_error = false;
    bool nonzero_write_active = false;
    std::string reason = "idle";
    std::string error;
    double left_rpm = 0.0;
    double right_rpm = 0.0;
};

class MotionWriteController {
public:
    MotionWriterDispatchSnapshot update(const MotionSafetyExecutorSnapshot &motion,
                                        MotionCommandWriter &writer,
                                        bool allow_writer_dispatch) {
        MotionWriterDispatchSnapshot s;
        s.timestamp_s = motion.timestamp_s;
        s.dispatch_enabled = allow_writer_dispatch;
        s.left_rpm = motion.target_left_rpm;
        s.right_rpm = motion.target_right_rpm;

        if (!allow_writer_dispatch) {
            s.reason = "dispatch_disabled";
            latest_ = s;
            update_stats(s);
            return latest_;
        }

        if (writer_fault_latched_) {
            if (writer_fault_zero_required_ || last_nonzero_dispatched_) return dispatch_zero(s, writer, "writer_fault_zero");
            s.writer_error = true;
            s.reason = "writer_fault_latched";
            s.error = last_error_;
            latest_ = s;
            update_stats(s);
            return latest_;
        }

        if (motion.command_duration_latched) {
            return dispatch_zero(s, writer, "command_duration_latched_zero");
        }

        if (motion.state == "WOULD_COMMAND" && motion.would_command) {
            return dispatch_rpm(s, writer, motion.target_left_rpm, motion.target_right_rpm, "would_command");
        }

        if (motion.state == "WOULD_ZERO" && motion.would_zero) {
            return dispatch_zero(s, writer, "would_zero");
        }

        if (motion.state == "BLOCKED" || motion.state == "FAULT") {
            if (last_nonzero_dispatched_) return dispatch_zero(s, writer, motion.state == "FAULT" ? "fault_zero" : "blocked_zero");
            s.reason = motion.state == "FAULT" ? "fault_no_active_command" : "blocked_no_active_command";
            latest_ = s;
            update_stats(s);
            return latest_;
        }

        if (motion.state == "IDLE") {
            if (last_nonzero_dispatched_) return dispatch_zero(s, writer, "idle_zero");
            s.reason = "idle";
            latest_ = s;
            update_stats(s);
            return latest_;
        }

        s.reason = "no_dispatch";
        latest_ = s;
        update_stats(s);
        return latest_;
    }

    MotionWriterDispatchSnapshot snapshot() const { return latest_; }

    MotionWriterDispatchRunStats run_stats(double) const { return stats_; }

private:
    MotionWriterDispatchSnapshot dispatch_zero(MotionWriterDispatchSnapshot s,
                                               MotionCommandWriter &writer,
                                               const std::string &reason) {
        s.attempted = true;
        s.left_rpm = 0.0;
        s.right_rpm = 0.0;
        MotionWriteResult r = writer.write_zero(s.timestamp_s);
        if (!r.ok) {
            s.writer_error = true;
            s.error = r.error.empty() ? "writer_error" : r.error;
            s.reason = "writer_error";
            writer_fault_latched_ = true;
            writer_fault_zero_required_ = true;
            last_error_ = s.error;
        } else {
            s.wrote_zero = true;
            s.reason = reason;
            last_nonzero_dispatched_ = false;
            writer_fault_zero_required_ = false;
        }
        latest_ = s;
        update_stats(s);
        return latest_;
    }

    MotionWriterDispatchSnapshot dispatch_rpm(MotionWriterDispatchSnapshot s,
                                              MotionCommandWriter &writer,
                                              double left_rpm,
                                              double right_rpm,
                                              const std::string &reason) {
        s.attempted = true;
        s.left_rpm = left_rpm;
        s.right_rpm = right_rpm;
        MotionWriteResult r = writer.write_wheel_rpm(s.timestamp_s, left_rpm, right_rpm);
        if (!r.ok) {
            s.writer_error = true;
            s.error = r.error.empty() ? "writer_error" : r.error;
            s.reason = "writer_error";
            writer_fault_latched_ = true;
            writer_fault_zero_required_ = true;
            last_error_ = s.error;
            last_nonzero_dispatched_ = false;
        } else {
            s.wrote_rpm = true;
            s.reason = reason;
            last_nonzero_dispatched_ = std::fabs(left_rpm) > 1e-12 || std::fabs(right_rpm) > 1e-12;
            s.nonzero_write_active = last_nonzero_dispatched_;
        }
        latest_ = s;
        update_stats(s);
        return latest_;
    }

    void update_stats(const MotionWriterDispatchSnapshot &s) {
        stats_.dispatch_enabled_last = s.dispatch_enabled;
        if (s.wrote_zero) stats_.zero_write_count++;
        if (s.wrote_rpm) stats_.rpm_write_count++;
        if (s.writer_error) stats_.error_count++;
        stats_.last_left_rpm = s.left_rpm;
        stats_.last_right_rpm = s.right_rpm;
        stats_.last_error = s.error;
    }

    MotionWriterDispatchSnapshot latest_;
    MotionWriterDispatchRunStats stats_;
    bool last_nonzero_dispatched_ = false;
    bool writer_fault_latched_ = false;
    bool writer_fault_zero_required_ = false;
    std::string last_error_;
};

} // namespace robot_slamd
