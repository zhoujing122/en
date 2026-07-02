#include "config.hpp"
#include "mapping_supervisor.hpp"

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
    c.mapping_supervisor_startup_grace_s = 1.0;
    c.mapping_supervisor_min_ready_time_s = 1.0;
    c.mapping_supervisor_min_good_quality_score = 0.40;
    c.mapping_supervisor_degraded_quality_score = 0.25;
    c.mapping_supervisor_lost_quality_score = 0.10;
    c.mapping_supervisor_max_degraded_duration_s = 2.0;
    c.mapping_supervisor_max_no_data_duration_s = 2.0;
    c.mapping_supervisor_active_scan_after_low_quality_s = 1.0;
    c.mapping_supervisor_active_scan_after_low_update_s = 1.0;
    c.mapping_supervisor_encoder_error_degraded_threshold = 5;
    c.mapping_supervisor_imu_error_degraded_threshold = 5;
    c.mapping_supervisor_tof_unhealthy_degraded_threshold = 1;
    c.map_quality_min_tof_valid_ratio = 0.30;
    c.map_quality_min_map_update_rate_hz = 1.0;
    c.map_quality_min_occupied_cells = 5;
    return c;
}

robot_slamd::MapQualitySnapshot quality(double score, const std::string &level, bool active_scan, int valid_routes) {
    robot_slamd::MapQualitySnapshot q;
    q.timestamp_s = 0.0;
    q.quality_score = score;
    q.quality_level = level;
    q.tof_samples_total = 30;
    q.tof_valid_hits = static_cast<uint64_t>(valid_routes * 8);
    q.tof_rejected = 30 - q.tof_valid_hits;
    q.tof_valid_ratio = q.tof_valid_hits / 30.0;
    q.tof_reject_ratio = q.tof_rejected / 30.0;
    q.map_updates = q.tof_valid_hits;
    q.map_update_rate_hz = valid_routes > 0 ? 2.0 : 0.0;
    q.occupied_cells = valid_routes >= 2 ? 20 : 2;
    q.known_cells = valid_routes >= 2 ? 200 : 20;
    q.active_scan_recommended = active_scan;
    if (valid_routes >= 1) { q.front.samples = 10; q.front.valid_hits = 8; q.front.confidence_sum = 850; q.front.confidence_count = 10; }
    if (valid_routes >= 2) { q.left.samples = 10; q.left.valid_hits = 8; q.left.confidence_sum = 850; q.left.confidence_count = 10; }
    if (valid_routes >= 3) { q.right.samples = 10; q.right.valid_hits = 8; q.right.confidence_sum = 850; q.right.confidence_count = 10; }
    return q;
}

robot_slamd::MapQualitySnapshot no_data_quality() {
    robot_slamd::MapQualitySnapshot q;
    q.quality_score = 0.0;
    q.quality_level = "NO_DATA";
    q.reason = "no_tof_data";
    q.active_scan_recommended = true;
    return q;
}

robot_slamd::SupervisorSensorInput sensors(uint64_t loc_updates = 10) {
    robot_slamd::SupervisorSensorInput s;
    s.localization_updates = loc_updates;
    return s;
}

void drive_to_mapping(robot_slamd::MappingSupervisor &sup) {
    auto good = quality(0.90, "GOOD", false, 2);
    sup.update(1.0, no_data_quality(), sensors(0));
    sup.update(1.1, good, sensors(1));
    sup.update(2.2, good, sensors(5));
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = test_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid mapping supervisor config should pass");

    MappingSupervisor sup(cfg);
    expect(sup.snapshot().state == "INIT", "default supervisor state should be INIT");
    sup.update(0.5, no_data_quality(), sensors(0));
    expect(sup.snapshot().state == "INIT", "before startup grace supervisor stays INIT");
    sup.update(1.0, no_data_quality(), sensors(0));
    expect(sup.snapshot().state == "SENSOR_CHECK", "INIT should transition to SENSOR_CHECK after startup grace");
    sup.update(1.1, quality(0.90, "GOOD", false, 2), sensors(1));
    expect(sup.snapshot().state == "MAPPING_READY", "SENSOR_CHECK should transition to MAPPING_READY with data");
    sup.update(2.2, quality(0.90, "GOOD", false, 2), sensors(5));
    expect(sup.snapshot().state == "MAPPING", "MAPPING_READY should transition to MAPPING after ready time and good quality");

    MappingSupervisor active_sup(cfg);
    drive_to_mapping(active_sup);
    auto low = quality(0.35, "LOW", true, 2);
    active_sup.update(2.3, low, sensors(6));
    expect(active_sup.snapshot().state == "MAPPING", "low quality should not immediately leave MAPPING before active scan timer");
    active_sup.update(3.4, low, sensors(7));
    MappingSupervisorSnapshot active_snap = active_sup.snapshot();
    expect(active_snap.state == "ACTIVE_SCAN_RECOMMENDED", "sustained low quality should recommend active scan");
    expect(active_snap.mapping_allowed, "active scan recommendation remains observe-only and mapping_allowed stays true");
    active_sup.update(3.6, quality(0.90, "GOOD", false, 2), sensors(8));
    expect(active_sup.snapshot().state == "MAPPING", "ACTIVE_SCAN_RECOMMENDED should recover to MAPPING when quality recovers");

    MappingSupervisor degraded_sup(cfg);
    drive_to_mapping(degraded_sup);
    SupervisorSensorInput bad_encoder = sensors(6);
    bad_encoder.encoder_left_errors = 5;
    degraded_sup.update(2.4, quality(0.90, "GOOD", false, 2), bad_encoder);
    expect(degraded_sup.snapshot().state == "DEGRADED", "encoder error threshold should enter DEGRADED");
    degraded_sup.update(4.6, quality(0.20, "DEGRADED", true, 2), bad_encoder);
    expect(degraded_sup.snapshot().state == "LOST", "DEGRADED duration should time out to LOST");
    degraded_sup.update(4.8, quality(0.90, "GOOD", false, 2), sensors(10));
    expect(degraded_sup.snapshot().state == "SENSOR_CHECK", "LOST with recovered data should return conservatively to SENSOR_CHECK");

    MappingSupervisor no_data_sup(cfg);
    no_data_sup.update(1.0, no_data_quality(), sensors(0));
    no_data_sup.update(3.2, no_data_quality(), sensors(0));
    expect(no_data_sup.snapshot().state == "DEGRADED", "NO_DATA timeout in SENSOR_CHECK should enter DEGRADED");
    no_data_sup.update(5.5, no_data_quality(), sensors(0));
    expect(no_data_sup.snapshot().state == "LOST", "continued NO_DATA should enter LOST");

    MappingSupervisor sensor_sup(cfg);
    drive_to_mapping(sensor_sup);
    SupervisorSensorInput bad_imu = sensors(5);
    bad_imu.imu_gap_count = 5;
    sensor_sup.update(2.5, quality(0.90, "GOOD", false, 2), bad_imu);
    expect(sensor_sup.snapshot().state == "DEGRADED", "IMU error threshold should enter DEGRADED");

    MappingSupervisor tof_sup(cfg);
    drive_to_mapping(tof_sup);
    SupervisorSensorInput bad_tof = sensors(5);
    bad_tof.unhealthy_tof_routes = 1;
    tof_sup.update(2.5, quality(0.90, "GOOD", false, 2), bad_tof);
    expect(tof_sup.snapshot().state == "DEGRADED", "ToF unhealthy threshold should enter DEGRADED");

    Config zero_threshold = cfg;
    zero_threshold.mapping_supervisor_encoder_error_degraded_threshold = 0;
    zero_threshold.mapping_supervisor_imu_error_degraded_threshold = 0;
    zero_threshold.mapping_supervisor_tof_unhealthy_degraded_threshold = 0;
    MappingSupervisor zero_sup(zero_threshold);
    drive_to_mapping(zero_sup);
    SupervisorSensorInput ignored_errors = sensors(5);
    ignored_errors.encoder_left_errors = 100;
    ignored_errors.imu_gap_count = 100;
    ignored_errors.unhealthy_tof_routes = 3;
    zero_sup.update(2.5, quality(0.90, "GOOD", false, 2), ignored_errors);
    expect(zero_sup.snapshot().state == "MAPPING", "zero sensor thresholds should disable those degraded checks");

    Config disabled = cfg;
    disabled.mapping_supervisor_enabled = false;
    expect_no_throw([&] { validate_config(disabled); }, "mapping_supervisor.enabled=false should still validate");
    Config bad_log = cfg;
    bad_log.mapping_supervisor_log_hz = 0.0;
    expect_throw([&] { validate_config(bad_log); }, "mapping_supervisor.log_hz <= 0 should be invalid");
    Config bad_startup = cfg;
    bad_startup.mapping_supervisor_startup_grace_s = -0.1;
    expect_throw([&] { validate_config(bad_startup); }, "negative startup_grace_s should be invalid");
    Config bad_score = cfg;
    bad_score.mapping_supervisor_min_good_quality_score = 1.2;
    expect_throw([&] { validate_config(bad_score); }, "score threshold above 1 should be invalid");
    Config bad_lost = cfg;
    bad_lost.mapping_supervisor_lost_quality_score = 0.30;
    bad_lost.mapping_supervisor_degraded_quality_score = 0.25;
    expect_throw([&] { validate_config(bad_lost); }, "lost threshold above degraded threshold should be invalid");
    Config bad_degraded = cfg;
    bad_degraded.mapping_supervisor_degraded_quality_score = 0.50;
    bad_degraded.mapping_supervisor_min_good_quality_score = 0.40;
    expect_throw([&] { validate_config(bad_degraded); }, "degraded threshold above good threshold should be invalid");

    if (failures) {
        std::cerr << failures << " test failures\n";
        return 1;
    }
    std::cout << "test_mapping_supervisor ok\n";
    return 0;
}
