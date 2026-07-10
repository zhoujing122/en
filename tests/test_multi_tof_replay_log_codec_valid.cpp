#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_log_codec.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sample_log.hpp"

#include <iostream>
#include <string>

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
    MultiTofReplayLogCodec codec;
    const auto records = codec.decode_lines(MultiTofReplaySampleLog::valid_log_lines());
    int packet_count = 0;
    MultiTofReplayRecord packet;
    for (const auto &record : records) {
        if (record.kind == MultiTofReplayRecordKind::Packet) {
            packet_count++;
            packet = record;
        }
    }
    expect(packet_count == 1, "packet count 1");
    expect(packet.packet.has_front && packet.packet.has_left && packet.packet.has_right,
           "front left right present");
    expect(packet.packet.front.frame_id == "tof_front_frame", "front frame preserved");
    expect(packet.packet.left.frame_id == "tof_left_frame", "left frame preserved");
    expect(packet.packet.right.frame_id == "tof_right_frame", "right frame preserved");
    expect(packet.packet.left.mount_yaw_rad > 1.5, "left yaw preserved");
    expect(packet.packet.front.ranges_m.size() == 4, "ranges preserved");
    expect(packet.packet.has_imu && packet.packet.has_wheel, "imu wheel present");
    const auto encoded = codec.encode_record(packet.packet);
    const auto decoded = codec.decode_line(encoded, 7);
    expect(decoded.kind == MultiTofReplayRecordKind::Packet, "roundtrip packet");
    expect(decoded.packet.front.frame_id == packet.packet.front.frame_id,
           "roundtrip front frame");
    return failures ? 1 : 0;
}
