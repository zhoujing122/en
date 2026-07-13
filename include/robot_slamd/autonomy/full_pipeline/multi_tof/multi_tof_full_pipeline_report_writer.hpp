#pragma once

#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_types.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class MultiTofFullPipelineReportWriter {
public:
    std::string write_text(const MultiTofFullPipelineReport &report) const {
        std::ostringstream o;
        o << "multi_tof_full_pipeline_report\n"
          << "ok=" << (report.ok ? 1 : 0) << "\n"
          << "scenario=" << to_string(report.scenario) << "\n"
          << "fault=" << to_string(report.fault) << "\n"
          << "summary=" << report.summary << "\n"
          << "data_path=scalar three-ToF replay -> RobotSlamSensorPort -> Coordinator -> legacy scalar projection -> deterministic backend\n"
          << "native_fusion=not implemented\n"
          << "real_hardware=not accessed\n"
          << "motion=fake motion only\n"
          << "map=fake artifact only\n"
          << "pose_writeback=not attempted\n"
          << "full_pipeline_summary=" << report.full_pipeline.summary << "\n"
          << "last_replay_status=" << to_string(report.last_replay_status) << "\n"
          << "last_replay_fault=" << to_string(report.last_replay_fault) << "\n"
          << "last_replay_message=" << report.last_replay_message << "\n"
          << "replay_read_call_count=" << report.replay_read_call_count << "\n"
          << "successful_snapshot_count=" << report.successful_snapshot_count << "\n"
          << "rejected_snapshot_count=" << report.rejected_snapshot_count << "\n"
          << "end_of_replay_return_count=" << report.end_of_replay_return_count << "\n"
          << "multi_tof_snapshot_count=" << report.multi_tof_snapshot_count << "\n"
          << "degraded_snapshot_count=" << report.degraded_snapshot_count << "\n"
          << "front_present_count=" << report.front_present_count << "\n"
          << "left_present_count=" << report.left_present_count << "\n"
          << "right_present_count=" << report.right_present_count << "\n"
          << "legacy_projection_count=" << report.legacy_projection_count << "\n"
          << "maximum_legacy_projection_range_count=" << report.maximum_legacy_projection_range_count << "\n"
          << "scalar_tof_codec_used=" << (report.scalar_tof_codec_used ? 1 : 0) << "\n"
          << "echo_tag_preserved=" << (report.echo_tag_preserved ? 1 : 0) << "\n"
          << "confidence_preserved=" << (report.confidence_preserved ? 1 : 0) << "\n"
          << "effective_timestamp_preserved=" << (report.effective_timestamp_preserved ? 1 : 0) << "\n"
          << "synchronized_time_verified=" << (report.synchronized_time_verified ? 1 : 0) << "\n"
          << "multi_tof_reached_runner=" << (report.multi_tof_reached_runner ? 1 : 0) << "\n"
          << "multi_tof_reached_coordinator=" << (report.multi_tof_reached_coordinator ? 1 : 0) << "\n"
          << "legacy_projection_reached_backend=" << (report.legacy_projection_reached_backend ? 1 : 0) << "\n"
          << "native_multi_tof_backend_consumption=" << (report.native_multi_tof_backend_consumption ? 1 : 0) << "\n"
          << "phase_aware_sensor_consumption_verified=" << (report.phase_aware_sensor_consumption_verified ? 1 : 0) << "\n"
          << "stale_snapshot_reused=" << (report.stale_snapshot_reused ? 1 : 0) << "\n"
          << "forward_seen=" << (report.forward_seen ? 1 : 0) << "\n"
          << "backward_seen=" << (report.backward_seen ? 1 : 0) << "\n"
          << "fake_motion_only=" << (report.fake_motion_only ? 1 : 0) << "\n"
          << "real_hardware_accessed=" << (report.real_hardware_accessed ? 1 : 0) << "\n"
          << "real_motion_attempted=" << (report.real_motion_attempted ? 1 : 0) << "\n"
          << "real_map_write_attempted=" << (report.real_map_write_attempted ? 1 : 0) << "\n"
          << "real_pose_writeback_attempted=" << (report.real_pose_writeback_attempted ? 1 : 0) << "\n"
          << "warning=multi_tof_pipeline_uses_legacy_scalar_projection_for_deterministic_backend\n";
        return o.str();
    }
};

} // namespace robot_slamd
