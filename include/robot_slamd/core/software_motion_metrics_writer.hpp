#pragma once
#include "robot_slamd/core/common.hpp"

namespace robot_slamd {

inline void write_software_motion_metrics(std::ostream &o,
                                          const RunMetrics &m,
                                          const Config &cfg) {
    o << "motion_writer_backend,"
      << (cfg.motion_execution_writer_backend == "software_direction_speed_test_only" ? 1 : 0)
      << "\n";
    o << "software_motion_last_direction," << m.software_motion.last_direction << "\n";
    o << "software_motion_last_speed_normalized," << m.software_motion.last_speed_normalized << "\n";
    o << "software_motion_last_ttl_s," << m.software_motion.last_ttl_s << "\n";
    o << "software_motion_last_send_ok," << (m.software_motion.last_send_ok ? 1 : 0) << "\n";
    o << "software_motion_last_accepted," << (m.software_motion.last_accepted ? 1 : 0) << "\n";
    o << "software_motion_send_count," << m.software_motion.send_count << "\n";
    o << "software_motion_reject_count," << m.software_motion.reject_count << "\n";
    o << "software_motion_error_count," << m.software_motion.error_count << "\n";
    o << "software_motion_stop_count," << m.software_motion.stop_count << "\n";
    o << "software_motion_turn_left_count," << m.software_motion.turn_left_count << "\n";
    o << "software_motion_turn_right_count," << m.software_motion.turn_right_count << "\n";
    o << "software_motion_forward_block_count," << m.software_motion.forward_block_count << "\n";
    o << "software_motion_backward_block_count," << m.software_motion.backward_block_count << "\n";
    o << "software_motion_validation_error_count," << m.software_motion.validation_error_count << "\n";
    o << "m2b1_preflight_enabled_last,"
      << (m.software_motion.m2b1_preflight_enabled_last ? 1 : 0) << "\n";
    o << "m2b1_preflight_ok_last,"
      << (m.software_motion.m2b1_preflight_ok_last ? 1 : 0) << "\n";
    o << "m2b1_preflight_error_count," << m.software_motion.m2b1_preflight_error_count << "\n";
    o << "m2b1_manual_arm_requested_last,"
      << (m.software_motion.m2b1_manual_arm_requested_last ? 1 : 0) << "\n";
    o << "m2b1_manual_arm_armed_last,"
      << (m.software_motion.m2b1_manual_arm_armed_last ? 1 : 0) << "\n";
    o << "m2b1_manual_arm_reject_count," << m.software_motion.m2b1_manual_arm_reject_count << "\n";
    o << "software_motion_transport_backend,"
      << (cfg.motion_execution_software_motion_production_transport_backend == "loopback_shadow" ? 1 : 0)
      << "\n";
    o << "software_motion_loopback_shadow_mode_last,"
      << (cfg.motion_execution_software_motion_loopback_shadow_mode ? 1 : 0) << "\n";
    o << "software_motion_loopback_send_count," << m.software_motion.loopback_send_count << "\n";
    o << "software_motion_loopback_reject_count," << m.software_motion.loopback_reject_count << "\n";
    o << "m2b1_ready_for_lifted_live_test_last,"
      << (m.software_motion.m2b1_ready_for_lifted_live_test_last ? 1 : 0) << "\n";
    o << "m2b1_preflight_mode_last,"
      << (cfg.motion_execution_m2b1_preflight_mode == "lifted_direction_probe" ? 0 : 1) << "\n";
    o << "m2b1_direction_convention_pending_last,"
      << (m.software_motion.m2b1_direction_convention_pending_last ? 1 : 0) << "\n";
    o << "m2b1_direction_probe_ready_last,"
      << (m.software_motion.m2b1_direction_probe_ready_last ? 1 : 0) << "\n";
    o << "m2b1_direction_probe_reject_count,"
      << m.software_motion.m2b1_direction_probe_reject_count << "\n";
    o << "m2b1_confirmed_live_ready_last,"
      << (m.software_motion.m2b1_confirmed_live_ready_last ? 1 : 0) << "\n";
    o << "m2b1_confirmed_live_reject_count,"
      << m.software_motion.m2b1_confirmed_live_reject_count << "\n";
}

} // namespace robot_slamd
