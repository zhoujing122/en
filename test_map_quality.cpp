#include "config.hpp"
#include "grid.hpp"
#include "map_quality.hpp"
#include "tof.hpp"

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

robot_slamd::TofFilterResult valid_hit() {
    robot_slamd::TofFilterResult r;
    r.map_update = true;
    r.hit = true;
    r.range_m = 1.0;
    r.filtered_confidence = 90;
    r.decision = "hit_update";
    return r;
}

robot_slamd::TofFilterResult rejected(const char *decision) {
    robot_slamd::TofFilterResult r;
    r.map_update = false;
    r.hit = false;
    r.range_m = 1.0;
    r.filtered_confidence = 0;
    r.decision = decision;
    return r;
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg;
    cfg.map_quality_min_occupied_cells = 0;
    cfg.map_quality_min_map_update_rate_hz = 0.0;
    cfg.map_quality_min_tof_valid_ratio = 0.30;
    cfg.map_quality_window_s = 5.0;
    cfg.map_quality_low_quality_score_threshold = 0.40;
    cfg.map_quality_degraded_score_threshold = 0.25;

    MapQuality empty(cfg);
    MapQualitySnapshot empty_s = empty.snapshot(0.0);
    expect(empty_s.quality_level == "NO_DATA", "empty window should be NO_DATA");
    expect(empty_s.quality_score == 0.0, "empty window score should be zero");
    expect(empty_s.active_scan_recommended, "empty window should recommend active scan");

    MapQuality good(cfg);
    GridStats gst;
    gst.total_cells = 100;
    gst.occupied_cells = 10;
    gst.free_cells = 20;
    gst.known_cells = 30;
    gst.unknown_cells = 70;
    good.update_grid_stats(gst);
    good.record_tof_result(1.0, {0, "front", 1000, 0, 90}, valid_hit());
    good.record_map_update(1.0, true, true, true, false);
    good.record_tof_result(1.1, {0, "left", 1000, 0, 95}, valid_hit());
    good.record_map_update(1.1, true, true, true, false);
    MapQualitySnapshot good_s = good.snapshot(1.2);
    expect(good_s.tof_samples_total == 2, "valid samples should be counted");
    expect(good_s.tof_valid_hits == 2, "valid hits should be counted");
    expect(good_s.front.samples == 1 && good_s.left.samples == 1, "front/left route samples should be counted");
    expect(good_s.front.valid_hits == 1 && good_s.left.valid_hits == 1, "front/left route valid hits should be counted");
    expect(good_s.tof_valid_ratio == 1.0, "valid ratio should be 1.0");
    expect(good_s.quality_level == "GOOD", "good window should be GOOD");
    expect(!good_s.active_scan_recommended, "good window should not recommend active scan");

    MapQuality low(cfg);
    low.record_tof_result(1.0, {0, "right", 1000, 0, 50}, rejected("rejected_confidence"));
    MapQualitySnapshot low_s = low.snapshot(1.1);
    expect(low_s.tof_low_confidence == 1, "low confidence should be counted");
    expect(low_s.tof_rejected == 1, "rejected sample should be counted");
    expect(low_s.tof_reject_ratio == 1.0, "reject ratio should be 1.0");
    expect(low_s.active_scan_recommended, "low confidence window should recommend active scan");

    Config strict = cfg;
    strict.map_quality_min_tof_valid_ratio = 0.80;
    strict.map_quality_min_map_update_rate_hz = 1.0;
    strict.map_quality_min_occupied_cells = 5;
    MapQuality degraded(strict);
    GridStats poor_grid;
    poor_grid.total_cells = 100;
    degraded.update_grid_stats(poor_grid);
    degraded.record_tof_result(1.0, {0, "front", 1000, 0, 90}, valid_hit());
    degraded.record_map_update(1.0, true, false, true, false);
    degraded.record_tof_result(1.1, {0, "front", 1000, 0, 0}, rejected("invalid_confidence"));
    degraded.record_map_update(1.1, false, false, false, false);
    MapQualitySnapshot degraded_s = degraded.snapshot(1.2);
    expect(degraded_s.quality_level == "DEGRADED" || degraded_s.quality_level == "LOW", "poor window should not be GOOD");
    expect(degraded_s.active_scan_recommended, "poor window should recommend active scan");

    Config filter_cfg;
    TofFilter filter(filter_cfg);
    TofSample invalid{0, "front", 1000, 0, 0};
    TofFilterResult invalid_res = filter.process(invalid);
    expect(!invalid_res.map_update, "confidence=0 with synthetic range_status=0 must not update map");
    expect(invalid_res.decision == "invalid_confidence", "confidence=0 should be invalid_confidence, not free_only");
    expect(invalid.range_status == 0, "confidence=0 test sample keeps synthetic range_status=0");

    ChunkedGrid grid(filter_cfg);
    TofSample low_conf{0, "front", 1000, 0, 50};
    TofFilterResult low_conf_res = filter.process(low_conf);
    if (low_conf_res.map_update) {
        grid.update_tof_ray({0.0, 0.0, 0.0}, filter_cfg.tof_extrinsics.at("front"), low_conf_res.range_m, low_conf_res.hit, filter_cfg.log_odds_free, 0.0, filter_cfg);
    }
    expect(grid.stats(filter_cfg).chunks == 0, "low confidence should not create free-clearing map chunks");

    ChunkedGrid stats_grid(filter_cfg);
    auto cell = stats_grid.cell_for_world_public(0.0, 0.0);
    stats_grid.add_log_odds_world(0.0, 0.0, filter_cfg.log_odds_occ);
    double before = stats_grid.log_odds_cell(cell.first, cell.second);
    GridStats st1 = stats_grid.stats(filter_cfg);
    GridStats st2 = stats_grid.stats(filter_cfg);
    double after = stats_grid.log_odds_cell(cell.first, cell.second);
    expect(before == after, "GridStats should not modify map cells");
    expect(st1.total_cells == static_cast<uint64_t>(filter_cfg.chunk_cells * filter_cfg.chunk_cells), "GridStats total_cells should cover allocated chunks only");
    expect(st1.occupied_cells == st2.occupied_cells && st1.unknown_cells == st2.unknown_cells, "GridStats should be stable across repeated calls");

    Config valid_cfg;
    expect_no_throw([&] { validate_config(valid_cfg); }, "default map_quality config should pass");
    Config disabled = valid_cfg;
    disabled.map_quality_enabled = false;
    expect_no_throw([&] { validate_config(disabled); }, "map_quality.enabled=false should pass");
    Config bad_log = valid_cfg;
    bad_log.map_quality_log_hz = 0.0;
    expect_throw([&] { validate_config(bad_log); }, "map_quality.log_hz <= 0 should be invalid");
    Config bad_window = valid_cfg;
    bad_window.map_quality_window_s = 0.0;
    expect_throw([&] { validate_config(bad_window); }, "map_quality.window_s <= 0 should be invalid");
    Config bad_ratio_low = valid_cfg;
    bad_ratio_low.map_quality_min_tof_valid_ratio = -0.1;
    expect_throw([&] { validate_config(bad_ratio_low); }, "map_quality ratio below 0 should be invalid");
    Config bad_ratio_high = valid_cfg;
    bad_ratio_high.map_quality_min_tof_valid_ratio = 1.1;
    expect_throw([&] { validate_config(bad_ratio_high); }, "map_quality ratio above 1 should be invalid");
    Config bad_threshold = valid_cfg;
    bad_threshold.map_quality_degraded_score_threshold = 0.6;
    bad_threshold.map_quality_low_quality_score_threshold = 0.4;
    expect_throw([&] { validate_config(bad_threshold); }, "degraded threshold above low threshold should be invalid");

    if (failures) {
        std::cerr << failures << " test failures\n";
        return 1;
    }
    std::cout << "test_map_quality ok\n";
    return 0;
}
