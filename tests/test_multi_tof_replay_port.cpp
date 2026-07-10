#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_port.hpp"
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
    MultiTofSyncOptions sync;
    sync.enabled = true;
    MultiTofSnapshotBuildOptions snapshot;
    snapshot.enabled = true;
    MultiTofReplayOptions replay;
    MultiTofReplayPort disabled(codec.decode_lines(MultiTofReplaySampleLog::valid_log_lines()),
                                {}, sync, snapshot, replay);
    expect(!disabled.ready(), "disabled not ready");
    replay.enabled = true;
    MultiTofReplayPort empty({}, {}, sync, snapshot, replay);
    expect(!empty.ready(), "empty not ready");
    MultiTofReplayPort comments(codec.decode_lines({"# comment"}), {}, sync, snapshot, replay);
    expect(!comments.ready(), "no packet not ready");
    MultiTofReplayPort invalid(codec.decode_lines(MultiTofReplaySampleLog::invalid_numeric_log_lines()),
                               {}, sync, snapshot, replay);
    expect(!invalid.ready(), "invalid record not ready");
    MultiTofReplayPort valid(codec.decode_lines(MultiTofReplaySampleLog::valid_log_lines()),
                             {}, sync, snapshot, replay);
    expect(valid.ready(), "valid ready");
    auto result = valid.latest_snapshot(100.0);
    expect(result.ok && result.snapshot.has_multi_tof, "latest snapshot ok");
    result = valid.latest_snapshot(100.0);
    expect(!result.ok && result.fault == MultiTofReplayFault::EndOfReplay,
           "end of replay");
    MultiTofReplayPort bad(codec.decode_lines(MultiTofReplaySampleLog::pairwise_sync_failure_log_lines()),
                           {}, sync, snapshot, replay);
    result = bad.latest_snapshot(100.0);
    expect(!result.ok && !result.snapshot.has_multi_tof,
           "fail does not reuse snapshot");
    return failures ? 1 : 0;
}
