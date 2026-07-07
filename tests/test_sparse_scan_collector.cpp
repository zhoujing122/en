#include "robot_slamd/config/config.hpp"
#include "robot_slamd/active_scan/sparse_scan_collector.hpp"

#include <functional>
#include <iostream>

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
        std::cerr << "FAIL: expected throw: " << message << "\n";
        failures++;
    } catch (const std::exception &) {
    }
}

void expect_no_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
    } catch (const std::exception &e) {
        std::cerr << "FAIL: unexpected throw: " << message << ": " << e.what() << "\n";
        failures++;
    }
}

robot_slamd::Config test_config() {
    robot_slamd::Config c;
    c.sparse_scan_log_hz = 20.0;
    c.sparse_scan_min_yaw_rate_dps = 3.0;
    c.sparse_scan_max_yaw_rate_dps = 90.0;
    c.sparse_scan_max_linear_speed_mps = 0.08;
    c.sparse_scan_min_range_m = 0.02;
    c.sparse_scan_max_range_m = 10.0;
    c.sparse_scan_min_confidence = 70;
    c.sparse_scan_bin_angle_deg = 10.0;
    c.sparse_scan_min_session_angle_deg = 10.0;
    c.sparse_scan_useful_session_angle_deg = 20.0;
    c.sparse_scan_complete_session_angle_deg = 30.0;
    c.sparse_scan_session_timeout_s = 1.0;
    c.sparse_scan_max_gap_s = 0.5;
    c.sparse_scan_cooldown_s = 0.2;
    c.sparse_scan_keep_invalid_samples = true;
    c.sparse_scan_compute_hit_points = true;
    c.sparse_scan_max_samples_per_session = 2000;
    return c;
}

robot_slamd::FilteredTofSample tof(const std::string &id, double range_m, int confidence, bool map_update = true, bool hit = true) {
    robot_slamd::FilteredTofSample t;
    t.timestamp_us = 1000;
    t.id = id;
    t.range_m = range_m;
    t.confidence = confidence;
    t.range_status = 0;
    t.map_update = map_update;
    t.hit = hit;
    t.decision = map_update ? "hit_update" : "filtered_rejected";
    return t;
}

robot_slamd::SparseScanInput input(double t, double yaw_deg = 0.0) {
    robot_slamd::SparseScanInput in;
    in.timestamp_s = t;
    in.timestamp_us = static_cast<uint64_t>(t * 1000000.0);
    in.pose = {0.0, 0.0, robot_slamd::deg2rad(yaw_deg)};
    in.linear_speed_mps = 0.0;
    in.yaw_rate_radps = robot_slamd::deg2rad(20.0);
    in.localization_valid = true;
    in.map_quality.quality_score = 0.5;
    in.supervisor.state = "MAPPING";
    in.supervisor.mapping_allowed = true;
    in.active_scan.state = "OBSERVING_ROTATION";
    in.active_scan.scan_recommended = true;
    in.active_scan_command.state = "IDLE";
    in.tof_samples = {tof("front", 1.0, 90), tof("left", 1.0, 85), tof("right", 1.0, 80)};
    return in;
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = test_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid sparse_scan config should pass");

    SparseScanCollector collector(cfg);
    expect(collector.state() == SparseScanState::IDLE, "default sparse collector should be IDLE");
    collector.update(input(0.0));
    expect(collector.state() == SparseScanState::COLLECTING, "active scan observation should start COLLECTING");
    auto samples = collector.drain_samples();
    expect(samples.size() == 3, "COLLECTING should emit one sample per ToF route");
    expect(std::fabs(samples[0].sensor_x_m - 0.10) < 1e-9, "front sensor x transform should use extrinsic x");
    expect(std::fabs(samples[0].sensor_y_m) < 1e-9, "front sensor y transform should use extrinsic y");
    expect(std::fabs(samples[0].hit_x_m - 1.10) < 1e-9, "front hit_x should be sensor_x + range");
    expect(std::fabs(samples[0].hit_y_m) < 1e-9, "front hit_y should remain zero at yaw 0");
    expect(std::fabs(samples[1].sensor_y_m - 0.08) < 1e-9, "left sensor y transform should use extrinsic y");
    expect(std::fabs(samples[1].global_bearing_deg - 90.0) < 1e-9, "left bearing should be 90 degrees at base yaw 0");

    SparseScanCollector command_trigger(cfg);
    auto cmd_in = input(0.0);
    cmd_in.active_scan.state = "IDLE";
    cmd_in.active_scan.scan_recommended = false;
    cmd_in.active_scan_command.state = "VERIFYING_ROTATION";
    command_trigger.update(cmd_in);
    expect(command_trigger.state() == SparseScanState::COLLECTING, "command VERIFYING_ROTATION should trigger collection");

    SparseScanCollector low_yaw(cfg);
    auto slow = input(0.0);
    slow.yaw_rate_radps = deg2rad(1.0);
    low_yaw.update(slow);
    expect(low_yaw.state() == SparseScanState::IDLE, "yaw rate below min should not start collection");

    SparseScanCollector high_yaw(cfg);
    high_yaw.update(input(0.0));
    auto too_fast_yaw = input(0.1);
    too_fast_yaw.yaw_rate_radps = deg2rad(120.0);
    high_yaw.update(too_fast_yaw);
    expect(high_yaw.state() == SparseScanState::ABORTED, "yaw rate above max should abort active session");
    expect(high_yaw.latest_summary().reason == "yaw_rate_too_high", "high yaw abort reason should be logged");

    SparseScanCollector high_linear(cfg);
    high_linear.update(input(0.0));
    auto fast = input(0.1);
    fast.linear_speed_mps = 0.20;
    high_linear.update(fast);
    expect(high_linear.state() == SparseScanState::ABORTED, "linear speed above max should abort active session");
    expect(high_linear.latest_summary().reason == "linear_speed_too_high", "linear abort reason should be logged");

    SparseScanCollector timeout(cfg);
    timeout.update(input(0.0));
    timeout.drain_samples();
    timeout.update(input(1.2));
    expect(timeout.state() == SparseScanState::ABORTED, "session timeout should abort active session");
    expect(timeout.latest_summary().reason == "session_timeout", "timeout reason should be logged");

    SparseScanCollector gap(cfg);
    gap.update(input(0.0));
    gap.drain_samples();
    auto no_tof = input(0.7);
    no_tof.tof_samples.clear();
    gap.update(no_tof);
    expect(gap.state() == SparseScanState::ABORTED, "max gap since valid sample should abort active session");
    expect(gap.latest_summary().reason == "max_gap_timeout", "max gap reason should be logged");

    SparseScanCollector wrap(cfg);
    auto w0 = input(0.0, 179.0);
    wrap.update(w0);
    wrap.drain_samples();
    auto w1 = input(0.1, -179.0);
    wrap.update(w1);
    auto ws = wrap.drain_samples();
    expect(!ws.empty(), "wrap update should emit samples");
    expect(ws[0].observed_yaw_delta_deg > 1.5 && ws[0].observed_yaw_delta_deg < 2.5, "yaw wrap 179 to -179 should count about 2 degrees");

    SparseScanCollector complete(cfg);
    complete.update(input(0.0, 0.0));
    complete.drain_samples();
    complete.update(input(0.1, 35.0));
    expect(complete.state() == SparseScanState::COMPLETED, "complete angle should complete sparse scan session");
    auto bins = complete.drain_bins();
    auto summaries = complete.drain_summaries();
    expect(!bins.empty(), "completed session should drain bins");
    expect(summaries.size() == 1, "completed session should drain one summary");
    expect(summaries[0].complete, "summary should mark complete session");
    expect(summaries[0].useful, "complete session should also be useful");
    expect(summaries[0].valid_samples >= 6, "valid samples should accumulate across updates");
    expect(complete.drain_bins().empty() && complete.drain_summaries().empty(), "drains should not return old data twice");

    SparseScanCollector useful(cfg);
    useful.update(input(0.0, 0.0));
    useful.drain_samples();
    useful.update(input(0.1, 25.0));
    useful.drain_samples();
    auto cleared = input(0.2, 25.0);
    cleared.active_scan.state = "IDLE";
    cleared.active_scan.scan_recommended = false;
    cleared.yaw_rate_radps = 0.0;
    useful.update(cleared);
    auto useful_summary = useful.latest_summary();
    expect(useful.state() == SparseScanState::COMPLETED, "useful angle plus request cleared should complete session");
    expect(useful_summary.useful && !useful_summary.complete, "useful completion should not require complete threshold");

    SparseScanCollector invalid(cfg);
    auto invalid_in = input(0.0);
    invalid_in.tof_samples = {tof("front", 1.0, 0, true, true)};
    invalid.update(invalid_in);
    auto invalid_samples = invalid.drain_samples();
    expect(invalid_samples.size() == 1, "invalid sample should be retained when keep_invalid_samples=true");
    expect(!invalid_samples[0].valid, "low confidence retained sample should be invalid");
    invalid.update(input(0.1, 35.0));
    auto invalid_summary = invalid.latest_summary();
    expect(invalid_summary.invalid_samples == 1, "invalid sample should count as invalid");

    Config bin_cfg = cfg;
    bin_cfg.sparse_scan_complete_session_angle_deg = 5.0;
    bin_cfg.sparse_scan_useful_session_angle_deg = 5.0;
    bin_cfg.sparse_scan_min_session_angle_deg = 1.0;
    SparseScanCollector bin_select(bin_cfg);
    auto b0 = input(0.0, 0.0);
    b0.tof_samples = {tof("front", 0.4, 80), tof("front", 0.6, 90), tof("front", 0.3, 90)};
    bin_select.update(b0);
    bin_select.drain_samples();
    bin_select.update(input(0.1, 6.0));
    auto selected_bins = bin_select.drain_bins();
    bool found_front_bin = false;
    for (const auto &b : selected_bins) {
        if (b.bin_index == 0 && b.valid_count >= 3) {
            found_front_bin = true;
            expect(b.sensor_mask == 1, "front bin sensor mask should be 1");
            expect(std::fabs(b.selected_range_m - 0.3) < 1e-9, "bin selection should prefer confidence then nearer range");
            expect(b.selected_confidence == 90, "bin selected confidence should be highest confidence");
        }
    }
    expect(found_front_bin, "front bin should be present after finalizing");

    SparseScanRunStats stats = bin_select.run_stats(0.2);
    expect(stats.sessions_started == 1, "run stats should count sessions started");
    expect(stats.sessions_completed == 1, "run stats should count completed sessions");
    expect(stats.total_samples >= 3, "run stats should count total samples");
    expect(stats.valid_samples >= 3, "run stats should count valid samples");
    expect(stats.last_valid_bin_ratio > 0.0, "run stats should record valid bin ratio");

    Config disabled = cfg;
    disabled.sparse_scan_enabled = false;
    expect_no_throw([&] { validate_config(disabled); }, "sparse_scan.enabled=false should validate");
    SparseScanCollector disabled_collector(disabled);
    disabled_collector.update(input(0.0));
    expect(disabled_collector.state() == SparseScanState::DISABLED, "disabled collector should stay DISABLED");
    expect(disabled_collector.drain_samples().empty(), "disabled collector should not emit samples");

    Config capped = cfg;
    capped.sparse_scan_max_samples_per_session = 1;
    SparseScanCollector cap(capped);
    cap.update(input(0.0));
    expect(cap.latest_summary().reason == "max_samples_reached", "sample cap should finalize with max_samples_reached");
    expect(cap.run_stats(0.0).total_samples == 1, "sample cap should stop recording after configured limit");

    Config bad_log = cfg;
    bad_log.sparse_scan_log_hz = 0.0;
    expect_throw([&] { validate_config(bad_log); }, "sparse_scan.log_hz <= 0 should be invalid");
    Config bad_yaw = cfg;
    bad_yaw.sparse_scan_min_yaw_rate_dps = 10.0;
    bad_yaw.sparse_scan_max_yaw_rate_dps = 5.0;
    expect_throw([&] { validate_config(bad_yaw); }, "sparse_scan max yaw below min should be invalid");
    Config bad_linear = cfg;
    bad_linear.sparse_scan_max_linear_speed_mps = -0.1;
    expect_throw([&] { validate_config(bad_linear); }, "negative sparse_scan linear speed should be invalid");
    Config bad_range = cfg;
    bad_range.sparse_scan_max_range_m = bad_range.sparse_scan_min_range_m;
    expect_throw([&] { validate_config(bad_range); }, "sparse_scan max range <= min range should be invalid");
    Config bad_conf = cfg;
    bad_conf.sparse_scan_min_confidence = -1;
    expect_throw([&] { validate_config(bad_conf); }, "negative sparse_scan confidence should be invalid");
    Config bad_bin = cfg;
    bad_bin.sparse_scan_bin_angle_deg = 0.0;
    expect_throw([&] { validate_config(bad_bin); }, "sparse_scan bin angle <= 0 should be invalid");
    Config bad_bin_big = cfg;
    bad_bin_big.sparse_scan_bin_angle_deg = 100.0;
    expect_throw([&] { validate_config(bad_bin_big); }, "sparse_scan bin angle > 90 should be invalid");
    Config bad_complete = cfg;
    bad_complete.sparse_scan_complete_session_angle_deg = 5.0;
    bad_complete.sparse_scan_useful_session_angle_deg = 10.0;
    expect_throw([&] { validate_config(bad_complete); }, "sparse_scan complete below useful should be invalid");
    Config bad_useful = cfg;
    bad_useful.sparse_scan_useful_session_angle_deg = 5.0;
    bad_useful.sparse_scan_min_session_angle_deg = 10.0;
    expect_throw([&] { validate_config(bad_useful); }, "sparse_scan useful below min should be invalid");
    Config bad_timeout = cfg;
    bad_timeout.sparse_scan_session_timeout_s = 0.0;
    expect_throw([&] { validate_config(bad_timeout); }, "sparse_scan session timeout <= 0 should be invalid");
    Config bad_gap = cfg;
    bad_gap.sparse_scan_max_gap_s = 0.0;
    expect_throw([&] { validate_config(bad_gap); }, "sparse_scan max gap <= 0 should be invalid");
    Config bad_cooldown = cfg;
    bad_cooldown.sparse_scan_cooldown_s = -0.1;
    expect_throw([&] { validate_config(bad_cooldown); }, "negative sparse_scan cooldown should be invalid");
    Config bad_cap = cfg;
    bad_cap.sparse_scan_max_samples_per_session = 0;
    expect_throw([&] { validate_config(bad_cap); }, "sparse_scan max samples <= 0 should be invalid");

    if (failures) {
        std::cerr << failures << " test failures\n";
        return 1;
    }
    std::cout << "test_sparse_scan_collector ok\n";
    return 0;
}
