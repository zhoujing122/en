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
    auto rec = codec.decode_lines(MultiTofReplaySampleLog::invalid_numeric_log_lines()).front();
    expect(rec.kind == MultiTofReplayRecordKind::InvalidRecord,
           "invalid numeric invalid record");
    expect(rec.error.find("invalid_numeric") != std::string::npos,
           "invalid numeric error");
    rec = codec.decode_lines(MultiTofReplaySampleLog::missing_field_log_lines()).front();
    expect(rec.kind == MultiTofReplayRecordKind::InvalidRecord,
           "missing field invalid record");
    expect(rec.error.find("missing_field") != std::string::npos,
           "missing field error");
    rec = codec.decode_lines(MultiTofReplaySampleLog::invalid_bool_log_lines()).front();
    expect(rec.kind == MultiTofReplayRecordKind::InvalidRecord,
           "invalid bool invalid record");
    expect(rec.error.find("invalid_bool") != std::string::npos,
           "invalid bool error");
    rec = codec.decode_lines(MultiTofReplaySampleLog::empty_ranges_log_lines()).front();
    expect(rec.kind == MultiTofReplayRecordKind::InvalidRecord,
           "empty ranges invalid record");
    expect(rec.error.find("empty_ranges") != std::string::npos,
           "empty ranges error");
    rec = codec.decode_line("this is not valid", 42);
    expect(rec.kind == MultiTofReplayRecordKind::InvalidRecord,
           "parse error not comment");
    expect(rec.line_number == 42, "line number preserved");
    expect(rec.raw_line == "this is not valid", "raw line preserved");
    return failures ? 1 : 0;
}
