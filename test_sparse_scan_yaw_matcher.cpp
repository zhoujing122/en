#include "config.hpp"
#include "grid.hpp"
#include "sparse_scan_yaw_matcher.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_near(double actual, double expected, double tolerance, const char *message) {
    if (std::fabs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << " actual=" << actual << " expected=" << expected << "\n";
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

robot_slamd::Config matcher_config() {
    robot_slamd::Config c;
    c.sparse_scan_yaw_match_enabled = true;
    c.sparse_scan_yaw_match_log_hz = 10.0;
    c.sparse_scan_yaw_match_max_yaw_search_deg = 10.0;
    c.sparse_scan_yaw_match_coarse_step_deg = 1.0;
    c.sparse_scan_yaw_match_refine_enabled = true;
    c.sparse_scan_yaw_match_refine_window_deg = 1.5;
    c.sparse_scan_yaw_match_refine_step_deg = 0.25;
    c.sparse_scan_yaw_match_min_valid_samples = 5;
    c.sparse_scan_yaw_match_min_valid_bins = 5;
    c.sparse_scan_yaw_match_min_observed_yaw_delta_deg = 90.0;
    c.sparse_scan_yaw_match_min_valid_bin_ratio = 0.10;
    c.sparse_scan_yaw_match_occupied_search_radius_m = 0.03;
    c.sparse_scan_yaw_match_distance_penalty_scale_m = 0.03;
    c.sparse_scan_yaw_match_min_best_score = 0.20;
    c.sparse_scan_yaw_match_min_score_margin = 0.05;
    c.sparse_scan_yaw_match_min_inlier_ratio = 0.40;
    c.sparse_scan_yaw_match_max_curve_flatness = 0.99;
    c.sparse_scan_yaw_match_multimodal_check_enabled = true;
    c.sparse_scan_yaw_match_multimodal_peak_separation_deg = 3.0;
    c.sparse_scan_yaw_match_max_second_peak_ratio = 0.85;
    c.sparse_scan_yaw_match_max_samples_per_match = 300;
    return c;
}

robot_slamd::SparseScanSample sample(uint64_t scan_id, double y_m, double base_yaw_deg) {
    robot_slamd::SparseScanSample s;
    s.scan_id = scan_id;
    s.timestamp_us = scan_id * 1000;
    s.timestamp_s = static_cast<double>(scan_id);
    s.sensor_id = "front";
    s.base_x_m = 0.0;
    s.base_y_m = y_m;
    s.base_yaw_rad = robot_slamd::deg2rad(base_yaw_deg);
    s.range_m = 3.025;
    s.confidence = 90;
    s.range_status = 0;
    s.valid = true;
    s.hit = true;
    return s;
}

std::vector<double> sample_ys() {
    return {-0.375, -0.175, 0.025, 0.225, 0.425};
}

robot_slamd::SparseScanForMatching make_scan(uint64_t scan_id, double measured_yaw_bias_deg, int n = 5) {
    robot_slamd::SparseScanForMatching scan;
    scan.scan_id = scan_id;
    std::vector<double> ys = sample_ys();
    for (int i = 0; i < n; ++i) {
        double y = i < static_cast<int>(ys.size()) ? ys[static_cast<size_t>(i)] : -0.5 + 0.1 * i;
        scan.samples.push_back(sample(scan_id, y, measured_yaw_bias_deg));
        robot_slamd::SparseScanBin b;
        b.scan_id = scan_id;
        b.bin_index = i;
        b.bin_center_deg = i * 5.0;
        b.sample_count = 1;
        b.valid_count = 1;
        b.sensor_mask = 1;
        b.valid = true;
        scan.bins.push_back(b);
    }
    scan.summary.scan_id = scan_id;
    scan.summary.state = "COMPLETED";
    scan.summary.reason = "complete_session";
    scan.summary.total_samples = n;
    scan.summary.valid_samples = n;
    scan.summary.front_valid = n;
    scan.summary.observed_yaw_delta_deg = 120.0;
    scan.summary.total_bins = 36;
    scan.summary.valid_bins = n;
    scan.summary.valid_bin_ratio = static_cast<double>(n) / 36.0;
    scan.summary.useful = true;
    scan.summary.complete = true;
    return scan;
}

std::pair<double, double> true_front_hit(const robot_slamd::Config &, double base_y_m, double true_yaw_deg) {
    double yaw = robot_slamd::deg2rad(true_yaw_deg);
    double sx = std::cos(yaw) * 0.10;
    double sy = base_y_m + std::sin(yaw) * 0.10;
    return {sx + std::cos(yaw) * 3.025, sy + std::sin(yaw) * 3.025};
}

void add_hits(robot_slamd::ChunkedGrid &grid, const robot_slamd::Config &cfg, double true_yaw_deg, double delta) {
    for (double y : sample_ys()) {
        auto hit = true_front_hit(cfg, y, true_yaw_deg);
        grid.add_log_odds_world(hit.first, hit.second, delta);
    }
}

robot_slamd::SparseScanYawMatchSummary run_one(robot_slamd::SparseScanYawMatcher &matcher,
                                               double now_s,
                                               const robot_slamd::SparseScanForMatching &scan,
                                               const robot_slamd::ChunkedGrid &grid,
                                               const robot_slamd::MapQualitySnapshot &mq = {}) {
    robot_slamd::MappingSupervisorSnapshot supervisor;
    matcher.update(now_s, std::vector<robot_slamd::SparseScanForMatching>{scan}, grid, mq, supervisor);
    auto summaries = matcher.drain_summaries();
    expect(summaries.size() == 1, "matcher should emit one summary per completed scan");
    return summaries.empty() ? robot_slamd::SparseScanYawMatchSummary{} : summaries.front();
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = matcher_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid sparse_scan_yaw_match config should pass");

    SparseScanYawMatcher disabled([&] { Config c = cfg; c.sparse_scan_yaw_match_enabled = false; return c; }());
    ChunkedGrid empty_grid(cfg);
    auto scan = make_scan(1, 5.0);
    MapQualitySnapshot mq;
    mq.quality_score = 0.5;
    mq.quality_level = "GOOD";
    MappingSupervisorSnapshot supervisor;
    expect(!disabled.update(1.0, std::vector<SparseScanForMatching>{scan}, empty_grid, mq, supervisor), "disabled matcher should not match");
    expect(disabled.drain_summaries().empty() && disabled.drain_curve_rows().empty(), "disabled matcher should not emit logs");

    SparseScanYawMatcher insufficient(cfg);
    auto too_few = make_scan(2, 5.0, 3);
    ChunkedGrid grid_with_map(cfg);
    add_hits(grid_with_map, cfg, 0.0, cfg.log_odds_occ * 2.0);
    auto too_few_summary = run_one(insufficient, 1.0, too_few, grid_with_map, mq);
    expect(!too_few_summary.attempted && too_few_summary.reason == "insufficient_samples", "insufficient samples should reject before search");

    SparseScanYawMatcher insufficient_bins(cfg);
    auto few_bins = make_scan(3, 5.0);
    few_bins.summary.valid_bins = 2;
    auto few_bins_summary = run_one(insufficient_bins, 1.0, few_bins, grid_with_map, mq);
    expect(!few_bins_summary.attempted && few_bins_summary.reason == "insufficient_bins", "insufficient bins should reject before search");

    SparseScanYawMatcher insufficient_yaw(cfg);
    auto low_yaw = make_scan(4, 5.0);
    low_yaw.summary.observed_yaw_delta_deg = 20.0;
    auto low_yaw_summary = run_one(insufficient_yaw, 1.0, low_yaw, grid_with_map, mq);
    expect(!low_yaw_summary.attempted && low_yaw_summary.reason == "insufficient_yaw_coverage", "insufficient yaw coverage should reject before search");

    SparseScanYawMatcher matcher(cfg);
    auto good_summary = run_one(matcher, 2.0, make_scan(5, 5.0), grid_with_map, mq);
    expect(good_summary.attempted, "good scan should attempt matching");
    expect(good_summary.usable, "occupied wall scan should be usable");
    expect_near(good_summary.best_yaw_delta_deg, -5.0, 0.35, "measured +5 deg yaw bias should match -5 deg yaw_delta");
    expect(good_summary.best_score > good_summary.second_score, "best score should exceed second score");
    expect_near(good_summary.score_margin, good_summary.best_score - good_summary.second_score, 1e-9, "score margin should be best minus second");
    expect(good_summary.inlier_ratio >= 0.8, "nearest occupied reward should produce high inlier ratio");
    auto curve = matcher.drain_curve_rows();
    expect(!curve.empty(), "matcher should emit yaw curve rows");
    bool saw_refined = false;
    for (const auto &c : curve) {
        if (std::fabs(c.yaw_delta_deg + 4.75) < 1e-9) saw_refined = true;
        expect(c.scan_id == 5, "curve rows should keep scan_id");
    }
    expect(saw_refined, "refine search should emit quarter-degree candidates");

    Config stride_cfg = cfg;
    stride_cfg.sparse_scan_yaw_match_min_valid_samples = 0;
    stride_cfg.sparse_scan_yaw_match_max_samples_per_match = 3;
    ChunkedGrid stride_grid(stride_cfg);
    add_hits(stride_grid, stride_cfg, 0.0, stride_cfg.log_odds_occ * 2.0);
    SparseScanYawMatcher stride_matcher(stride_cfg);
    auto stride_summary = run_one(stride_matcher, 3.0, make_scan(6, 5.0, 10), stride_grid, mq);
    expect(stride_summary.tested_samples <= 3, "max_samples_per_match should deterministic-stride downsample samples");

    Config low_margin_cfg = cfg;
    low_margin_cfg.sparse_scan_yaw_match_min_score_margin = 2.0;
    SparseScanYawMatcher low_margin_matcher(low_margin_cfg);
    ChunkedGrid low_margin_grid(low_margin_cfg);
    add_hits(low_margin_grid, low_margin_cfg, 0.0, low_margin_cfg.log_odds_occ * 2.0);
    auto low_margin_summary = run_one(low_margin_matcher, 4.0, make_scan(7, 5.0), low_margin_grid, mq);
    expect(!low_margin_summary.usable && low_margin_summary.reason == "low_score_margin", "low margin should mark candidate unusable");

    Config flat_cfg = cfg;
    flat_cfg.sparse_scan_yaw_match_min_best_score = -1.0;
    flat_cfg.sparse_scan_yaw_match_min_score_margin = 0.0;
    flat_cfg.sparse_scan_yaw_match_min_inlier_ratio = 0.0;
    flat_cfg.sparse_scan_yaw_match_max_curve_flatness = 0.5;
    SparseScanYawMatcher flat_matcher(flat_cfg);
    ChunkedGrid flat_grid(flat_cfg);
    flat_grid.add_log_odds_world(-3.0, -3.0, flat_cfg.log_odds_occ * 2.0);
    auto flat_summary = run_one(flat_matcher, 5.0, make_scan(8, 5.0), flat_grid, mq);
    expect(!flat_summary.usable && flat_summary.reason == "curve_flat", "curve_flat gate should reject flat unknown-support curve");

    Config multimodal_cfg = cfg;
    multimodal_cfg.sparse_scan_yaw_match_min_score_margin = 0.0;
    multimodal_cfg.sparse_scan_yaw_match_max_curve_flatness = 1.0;
    multimodal_cfg.sparse_scan_yaw_match_max_second_peak_ratio = 0.70;
    SparseScanYawMatcher multimodal_matcher(multimodal_cfg);
    ChunkedGrid multimodal_grid(multimodal_cfg);
    add_hits(multimodal_grid, multimodal_cfg, 0.0, multimodal_cfg.log_odds_occ * 2.0);
    add_hits(multimodal_grid, multimodal_cfg, 10.0, multimodal_cfg.log_odds_occ * 2.0);
    auto multimodal_summary = run_one(multimodal_matcher, 6.0, make_scan(9, 5.0), multimodal_grid, mq);
    expect(!multimodal_summary.usable && multimodal_summary.reason == "multimodal", "secondary separated peak should mark curve multimodal");
    expect(multimodal_summary.peak_count >= 2, "multimodal curve should report multiple peaks");

    Config free_cfg = cfg;
    free_cfg.sparse_scan_yaw_match_min_best_score = 0.0;
    SparseScanYawMatcher free_matcher(free_cfg);
    ChunkedGrid free_grid(free_cfg);
    add_hits(free_grid, free_cfg, 0.0, free_cfg.log_odds_free * 10.0);
    free_grid.add_log_odds_world(-3.0, -3.0, free_cfg.log_odds_occ * 2.0);
    auto free_summary = run_one(free_matcher, 7.0, make_scan(10, 5.0), free_grid, mq);
    expect(free_summary.attempted && free_summary.best_score < 0.0, "free cell penalty should reduce score below zero");

    Config unknown_cfg = cfg;
    unknown_cfg.sparse_scan_yaw_match_min_best_score = 0.0;
    SparseScanYawMatcher unknown_matcher(unknown_cfg);
    ChunkedGrid unknown_grid(unknown_cfg);
    unknown_grid.add_log_odds_world(-3.0, -3.0, unknown_cfg.log_odds_occ * 2.0);
    auto unknown_summary = run_one(unknown_matcher, 8.0, make_scan(11, 5.0), unknown_grid, mq);
    expect(unknown_summary.attempted && unknown_summary.best_score < 0.0, "unknown/out-of-map penalty should reduce score below zero");

    ChunkedGrid read_only_grid(cfg);
    auto before = read_only_grid.stats(cfg);
    auto q = read_only_grid.query_world(100.0, 100.0, cfg);
    auto near = read_only_grid.nearest_occupied_distance(100.0, 100.0, 0.1, cfg);
    auto after = read_only_grid.stats(cfg);
    expect(q.state == GridQueryState::OUT_OF_MAP && !near.found, "empty grid query should report out of map and no occupied cell");
    expect(before.chunks == after.chunks && after.chunks == 0, "readonly grid query should not allocate chunks");

    auto stats = matcher.run_stats(0.0);
    expect(stats.attempts == 1 && stats.usable_count == 1, "run stats should count usable match");
    expect(stats.last_reason == "ok", "run stats should keep last reason");

    Config invalid = cfg;
    invalid.sparse_scan_yaw_match_log_hz = 0.0;
    expect_throw([&] { validate_config(invalid); }, "log_hz <= 0 should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_max_yaw_search_deg = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_yaw_search_deg <= 0 should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_coarse_step_deg = 11.0;
    expect_throw([&] { validate_config(invalid); }, "coarse_step > max search should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_refine_window_deg = -1.0;
    expect_throw([&] { validate_config(invalid); }, "negative refine window should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_refine_step_deg = 0.0;
    expect_throw([&] { validate_config(invalid); }, "refine step <= 0 should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_min_valid_samples = -1;
    expect_throw([&] { validate_config(invalid); }, "negative min_valid_samples should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_min_valid_bin_ratio = 1.1;
    expect_throw([&] { validate_config(invalid); }, "invalid valid_bin_ratio should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_occupied_search_radius_m = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative occupied_search_radius should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_distance_penalty_scale_m = 0.0;
    expect_throw([&] { validate_config(invalid); }, "distance_penalty_scale <= 0 should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_min_inlier_ratio = -0.1;
    expect_throw([&] { validate_config(invalid); }, "invalid min_inlier_ratio should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_max_curve_flatness = 1.1;
    expect_throw([&] { validate_config(invalid); }, "invalid max_curve_flatness should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_max_second_peak_ratio = 1.1;
    expect_throw([&] { validate_config(invalid); }, "invalid max_second_peak_ratio should fail");
    invalid = cfg;
    invalid.sparse_scan_yaw_match_max_samples_per_match = 0;
    expect_throw([&] { validate_config(invalid); }, "max_samples_per_match <= 0 should fail");

    if (failures) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    return 0;
}
