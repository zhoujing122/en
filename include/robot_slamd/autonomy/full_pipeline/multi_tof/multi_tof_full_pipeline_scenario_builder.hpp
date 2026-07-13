#pragma once

#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_types.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_log_codec.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sample_log.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_sample_data.hpp"

namespace robot_slamd {

class MultiTofFullPipelineScenarioBuilder {
public:
    MultiTofFullPipelineScenario build(
        MultiTofFullPipelineScenarioKind kind) const {
        MultiTofFullPipelineScenario scenario;
        scenario.kind = kind;
        scenario.name = to_string(kind);
        scenario.contract_options = default_contract_options();
        scenario.sync_options = default_sync_options();
        scenario.snapshot_options = default_snapshot_options();
        scenario.replay_options = default_replay_options();
        scenario.full_pipeline_scenario =
            FullAutonomousSlamScenarioBuilder().build(
                FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes);
        scenario.full_pipeline_scenario.name = scenario.name;
        scenario.full_pipeline_scenario.backend_options.min_valid_ranges = 1;
        scenario.full_pipeline_scenario.backend_options.min_valid_range_ratio = 1.0;
        scenario.full_pipeline_scenario.backend_options.max_range_m = 12.0;
        scenario.full_pipeline_scenario.pipeline_options.min_backend_accepted_updates = 2;

        switch (kind) {
        case MultiTofFullPipelineScenarioKind::NormalRequireAll:
        case MultiTofFullPipelineScenarioKind::ActiveScanRequired:
            scenario.records = records_for_valid_packets(8);
            return scenario;
        case MultiTofFullPipelineScenarioKind::AllowMissingRight:
            scenario.contract_options.require_right = false;
            scenario.contract_options.min_required_tof_count = 2;
            scenario.sync_options.degradation_mode =
                MultiTofDegradationMode::AllowMissingOne;
            scenario.sync_options.min_required_tof_count = 2;
            scenario.snapshot_options.min_required_tof_count = 2;
            scenario.records = records_for_missing_right_packets(8);
            return scenario;
        case MultiTofFullPipelineScenarioKind::MissingRightRequireAll:
            scenario.records = records_for_missing_right_packets(1);
            return scenario;
        case MultiTofFullPipelineScenarioKind::LowConfidenceFront:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(line_for(low_confidence_front_packet())));
            return scenario;
        case MultiTofFullPipelineScenarioKind::InvalidPayloadLength:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(invalid_payload_length_line()));
            return scenario;
        case MultiTofFullPipelineScenarioKind::InvalidHexPayload:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(invalid_hex_payload_line()));
            return scenario;
        case MultiTofFullPipelineScenarioKind::DistanceAboveProtocolMaximum:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(line_for(distance_above_max_packet())));
            return scenario;
        case MultiTofFullPipelineScenarioKind::PairwiseSyncFailure:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(
                    line_for(MultiTofSyncSampleData::front_left_sync_too_large_packet())));
            return scenario;
        case MultiTofFullPipelineScenarioKind::ImuSyncFailure:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(
                    line_for(MultiTofSyncSampleData::imu_sync_too_large_packet())));
            return scenario;
        case MultiTofFullPipelineScenarioKind::WheelSyncFailure:
            scenario.records = records_from_lines(
                invalid_then_valid_lines(
                    line_for(MultiTofSyncSampleData::wheel_sync_too_large_packet())));
            return scenario;
        case MultiTofFullPipelineScenarioKind::EndOfReplayEarly:
            scenario.records = records_for_valid_packets(1);
            return scenario;
        case MultiTofFullPipelineScenarioKind::BackendReject:
            scenario.records = records_for_valid_packets(8);
            scenario.full_pipeline_scenario.backend_options.min_valid_ranges = 2;
            scenario.full_pipeline_scenario.pipeline_options.min_backend_accepted_updates = 1;
            return scenario;
        case MultiTofFullPipelineScenarioKind::NullSensorPort:
        case MultiTofFullPipelineScenarioKind::NotReadySensorPort:
            scenario.records = records_for_valid_packets(1);
            return scenario;
        }
        return scenario;
    }

private:
    static MultiTofContractOptions default_contract_options() {
        MultiTofContractOptions options;
        options.require_front = true;
        options.require_left = true;
        options.require_right = true;
        options.require_unique_mount_ids = true;
        options.require_unique_frame_ids = true;
        options.require_request_timing = true;
        options.min_required_tof_count = 3;
        return options;
    }

    static MultiTofSyncOptions default_sync_options() {
        MultiTofSyncOptions options;
        options.enabled = true;
        options.require_raw_contract_pass = true;
        options.require_imu = true;
        options.require_wheel = true;
        options.degradation_mode = MultiTofDegradationMode::RequireAll;
        options.min_required_tof_count = 3;
        return options;
    }

    static MultiTofSnapshotBuildOptions default_snapshot_options() {
        MultiTofSnapshotBuildOptions options;
        options.enabled = true;
        options.require_sync_pass = true;
        options.populate_legacy_tof = true;
        options.require_legacy_primary = true;
        options.legacy_primary_mode = MultiTofLegacyPrimaryMode::Front;
        options.min_required_tof_count = 3;
        return options;
    }

    static MultiTofReplayOptions default_replay_options() {
        MultiTofReplayOptions options;
        options.enabled = true;
        options.reject_invalid_records = true;
        options.require_snapshot_build = true;
        options.time_mode = MultiTofReplayTimeMode::RecordPacketTime;
        return options;
    }

    static std::vector<std::string> lines_for_valid_packets(int count) {
        std::vector<std::string> lines;
        for (int i = 0; i < count; ++i) {
            auto packet = MultiTofSyncSampleData::valid_synced_packet();
            packet.front.echo_tag_u48 += static_cast<uint64_t>(i * 3 + 1);
            packet.left.echo_tag_u48 += static_cast<uint64_t>(i * 3 + 2);
            packet.right.echo_tag_u48 += static_cast<uint64_t>(i * 3 + 3);
            shift_packet_time(packet, 0.100 * static_cast<double>(i));
            lines.push_back(line_for(packet));
        }
        return lines;
    }

    static std::vector<MultiTofReplayRecord> records_for_valid_packets(int count) {
        return records_from_lines(lines_for_valid_packets(count));
    }

    static std::vector<MultiTofReplayRecord>
    records_for_missing_right_packets(int count) {
        std::vector<std::string> lines;
        for (int i = 0; i < count; ++i) {
            auto packet = MultiTofSyncSampleData::valid_synced_packet();
            packet.has_right = false;
            shift_packet_time(packet, 0.100 * static_cast<double>(i));
            lines.push_back(line_for(packet));
        }
        return records_from_lines(lines);
    }

    static std::vector<MultiTofReplayRecord> records_from_lines(
        const std::vector<std::string> &lines) {
        return MultiTofReplayLogCodec().decode_lines(lines);
    }

    static std::string line_for(const MultiTofRawPacket &packet) {
        return MultiTofReplayLogCodec().encode_record(packet);
    }

    static std::vector<std::string> invalid_then_valid_lines(
        const std::string &invalid_line) {
        auto lines = lines_for_valid_packets(1);
        lines.insert(lines.begin(), invalid_line);
        return lines;
    }

    static MultiTofRawPacket low_confidence_front_packet() {
        auto packet = MultiTofSyncSampleData::valid_synced_packet();
        packet.front.confidence = 69;
        return packet;
    }

    static MultiTofRawPacket distance_above_max_packet() {
        auto packet = MultiTofSyncSampleData::valid_synced_packet();
        packet.front.distance_mm = 12001;
        return packet;
    }

    static std::string invalid_payload_length_line() {
        auto line = line_for(MultiTofSyncSampleData::valid_synced_packet());
        replace_key_value(line, "front_payload_hex", "front_payload_hex=112233440000AB0A");
        return line;
    }

    static std::string invalid_hex_payload_line() {
        auto line = line_for(MultiTofSyncSampleData::valid_synced_packet());
        replace_key_value(line, "front_payload_hex", "front_payload_hex=112233440000AB0A6Z");
        return line;
    }

    static void replace_key_value(
        std::string &line,
        const std::string &key,
        const std::string &replacement) {
        const std::string needle = " " + key + "=";
        const auto pos = line.find(needle);
        if (pos == std::string::npos) {
            return;
        }
        const auto end = line.find(" ", pos + 1);
        const auto count = end == std::string::npos ? std::string::npos : end - pos;
        line.replace(pos, count, " " + replacement);
    }

    static void shift_timing(RealSensorRequestTiming &timing, double delta_s) {
        timing.request_start_s += delta_s;
        timing.response_received_s += delta_s;
        timing.estimated_sample_time_s += delta_s;
    }

    static void shift_packet_time(MultiTofRawPacket &packet, double delta_s) {
        packet.packet_timestamp_s += delta_s;
        if (packet.has_front) {
            shift_timing(packet.front.timing, delta_s);
        }
        if (packet.has_left) {
            shift_timing(packet.left.timing, delta_s);
        }
        if (packet.has_right) {
            shift_timing(packet.right.timing, delta_s);
        }
        if (packet.has_imu) {
            packet.imu.timestamp_s += delta_s;
        }
        if (packet.has_wheel) {
            shift_timing(packet.wheel.timing, delta_s);
        }
    }
};

} // namespace robot_slamd
