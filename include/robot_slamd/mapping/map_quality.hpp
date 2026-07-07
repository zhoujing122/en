#pragma once
#include "robot_slamd/core/common.hpp"

namespace robot_slamd {

inline double map_quality_route_valid_ratio(const MapQualityRouteSnapshot &r) {
    return r.samples ? static_cast<double>(r.valid_hits) / static_cast<double>(r.samples) : 0.0;
}

inline double map_quality_route_confidence_mean(const MapQualityRouteSnapshot &r) {
    return r.confidence_count ? static_cast<double>(r.confidence_sum) / static_cast<double>(r.confidence_count) : 0.0;
}

class MapQuality {
public:
    explicit MapQuality(const Config &cfg) : cfg_(cfg) {}

    void record_tof_result(double now_s, const TofSample &sample, const TofFilterResult &result) {
        TofEvent ev;
        ev.t_s = now_s;
        ev.id = sample.id;
        ev.confidence = sample.confidence;
        ev.valid_hit = result.map_update && result.hit;
        ev.rejected = !result.map_update;
        tof_events_.push_back(ev);
        prune(now_s);
    }

    void record_map_update(double now_s, bool attempted, bool updated, bool hit, bool free_update) {
        MapEvent ev;
        ev.t_s = now_s;
        ev.attempted = attempted;
        ev.updated = updated;
        ev.hit = hit;
        ev.free_update = free_update;
        map_events_.push_back(ev);
        prune(now_s);
    }

    void update_grid_stats(const GridStats &stats) { grid_stats_ = stats; }

    bool should_log(double now_s) const {
        if (!cfg_.map_quality_enabled) return false;
        double period_s = 1.0 / cfg_.map_quality_log_hz;
        if (last_log_s_ < 0.0) return now_s >= period_s;
        return (now_s - last_log_s_) >= period_s;
    }

    MapQualitySnapshot snapshot(double now_s) {
        prune(now_s);
        MapQualitySnapshot s;
        s.timestamp_s = now_s;
        s.occupied_cells = grid_stats_.occupied_cells;
        s.free_cells = grid_stats_.free_cells;
        s.unknown_cells = grid_stats_.unknown_cells;
        s.known_cells = grid_stats_.known_cells;
        s.total_cells = grid_stats_.total_cells;
        s.map_alloc_failures = grid_stats_.map_alloc_failures;
        if (s.total_cells > 0) {
            s.occupied_ratio = static_cast<double>(s.occupied_cells) / static_cast<double>(s.total_cells);
            s.free_ratio = static_cast<double>(s.free_cells) / static_cast<double>(s.total_cells);
            s.known_ratio = static_cast<double>(s.known_cells) / static_cast<double>(s.total_cells);
        }

        uint64_t conf_sum = 0;
        uint64_t conf_count = 0;
        for (const auto &ev : tof_events_) {
            s.tof_samples_total++;
            if (ev.valid_hit) s.tof_valid_hits++;
            if (ev.confidence == 0) s.tof_invalid++;
            else if (ev.confidence < cfg_.confidence_min) s.tof_low_confidence++;
            if (ev.rejected) s.tof_rejected++;
            if (ev.confidence > 0) {
                conf_sum += static_cast<uint64_t>(ev.confidence);
                conf_count++;
            }
            MapQualityRouteSnapshot &route = route_snapshot(s, ev.id);
            route.samples++;
            if (ev.valid_hit) route.valid_hits++;
            if (ev.confidence > 0) {
                route.confidence_sum += static_cast<uint64_t>(ev.confidence);
                route.confidence_count++;
            }
        }
        if (s.tof_samples_total > 0) {
            s.tof_valid_ratio = static_cast<double>(s.tof_valid_hits) / static_cast<double>(s.tof_samples_total);
            s.tof_reject_ratio = static_cast<double>(s.tof_rejected) / static_cast<double>(s.tof_samples_total);
        }
        if (conf_count > 0) s.tof_confidence_mean = static_cast<double>(conf_sum) / static_cast<double>(conf_count);

        for (const auto &ev : map_events_) {
            if (ev.attempted) s.map_update_attempts++;
            if (ev.updated) s.map_updates++;
        }
        s.map_update_rate_hz = static_cast<double>(s.map_updates) / cfg_.map_quality_window_s;

        s.quality_score = compute_score(s);
        s.quality_level = quality_level(s);
        s.active_scan_recommended = active_scan_recommended(s);
        s.degraded = s.quality_level == "DEGRADED";
        s.reason = primary_reason(s);
        return s;
    }

    void mark_logged(double now_s, const MapQualitySnapshot &s) {
        last_log_s_ = now_s;
        run_stats_.score_last = s.quality_score;
        if (run_stats_.score_count == 0) run_stats_.score_min = s.quality_score;
        else run_stats_.score_min = std::min(run_stats_.score_min, s.quality_score);
        run_stats_.score_sum += s.quality_score;
        run_stats_.score_count++;
        if (s.quality_level == "LOW") run_stats_.low_count++;
        if (s.quality_level == "DEGRADED" || s.quality_level == "NO_DATA") run_stats_.degraded_count++;
        if (s.active_scan_recommended) run_stats_.active_scan_recommended_count++;
        run_stats_.tof_valid_ratio_last = s.tof_valid_ratio;
        run_stats_.tof_reject_ratio_last = s.tof_reject_ratio;
        run_stats_.map_update_rate_hz_last = s.map_update_rate_hz;
        run_stats_.map_known_cells_last = s.known_cells;
        run_stats_.map_occupied_cells_last = s.occupied_cells;
        run_stats_.map_alloc_failures = s.map_alloc_failures;
    }

    const MapQualityRunStats &run_stats() const { return run_stats_; }

private:
    struct TofEvent {
        double t_s = 0.0;
        std::string id;
        int confidence = 0;
        bool valid_hit = false;
        bool rejected = false;
    };
    struct MapEvent {
        double t_s = 0.0;
        bool attempted = false;
        bool updated = false;
        bool hit = false;
        bool free_update = false;
    };

    void prune(double now_s) {
        double cutoff = now_s - cfg_.map_quality_window_s;
        while (!tof_events_.empty() && tof_events_.front().t_s < cutoff) tof_events_.pop_front();
        while (!map_events_.empty() && map_events_.front().t_s < cutoff) map_events_.pop_front();
    }

    static MapQualityRouteSnapshot &route_snapshot(MapQualitySnapshot &s, const std::string &id) {
        if (id == "front") return s.front;
        if (id == "left") return s.left;
        return s.right;
    }

    double compute_score(const MapQualitySnapshot &s) const {
        if (s.tof_samples_total == 0) return 0.0;
        double score = 1.0;
        if (cfg_.map_quality_min_tof_valid_ratio > 0.0) {
            score *= clamp01(s.tof_valid_ratio / cfg_.map_quality_min_tof_valid_ratio);
        }
        if (cfg_.map_quality_min_map_update_rate_hz > 0.0 && s.map_update_rate_hz < cfg_.map_quality_min_map_update_rate_hz) {
            score *= clamp01(s.map_update_rate_hz / cfg_.map_quality_min_map_update_rate_hz);
        }
        if (s.occupied_cells < cfg_.map_quality_min_occupied_cells) score *= 0.5;
        if (s.map_alloc_failures > 0) score *= 0.5;
        if (valid_routes(s) < 2) score *= 0.6;
        return clamp01(score);
    }

    static int valid_routes(const MapQualitySnapshot &s) {
        int routes = 0;
        if (s.front.valid_hits > 0) routes++;
        if (s.left.valid_hits > 0) routes++;
        if (s.right.valid_hits > 0) routes++;
        return routes;
    }

    std::string quality_level(const MapQualitySnapshot &s) const {
        if (s.tof_samples_total == 0) return "NO_DATA";
        if (s.quality_score >= cfg_.map_quality_low_quality_score_threshold) return "GOOD";
        if (s.quality_score >= cfg_.map_quality_degraded_score_threshold) return "LOW";
        return "DEGRADED";
    }

    bool active_scan_recommended(const MapQualitySnapshot &s) const {
        if (s.quality_level == "LOW" || s.quality_level == "DEGRADED" || s.quality_level == "NO_DATA") return true;
        if (s.tof_valid_ratio < cfg_.map_quality_min_tof_valid_ratio) return true;
        if (s.map_update_rate_hz < cfg_.map_quality_min_map_update_rate_hz) return true;
        if (s.occupied_cells < cfg_.map_quality_min_occupied_cells) return true;
        if (valid_routes(s) < 2) return true;
        if (s.tof_reject_ratio > 0.80) return true;
        return false;
    }

    std::string primary_reason(const MapQualitySnapshot &s) const {
        if (s.tof_samples_total == 0) return "no_tof_data";
        if (s.map_alloc_failures > 0) return "map_alloc_failure";
        if (s.tof_valid_ratio < cfg_.map_quality_min_tof_valid_ratio) return "low_tof_valid_ratio";
        if (s.map_update_rate_hz < cfg_.map_quality_min_map_update_rate_hz) return "low_map_update_rate";
        if (s.occupied_cells < cfg_.map_quality_min_occupied_cells) return "too_few_occupied_cells";
        if (valid_routes(s) < 2) return "single_tof_route_only";
        if (s.tof_reject_ratio > 0.80) return "high_tof_reject_ratio";
        return "ok";
    }

    const Config &cfg_;
    std::deque<TofEvent> tof_events_;
    std::deque<MapEvent> map_events_;
    GridStats grid_stats_;
    double last_log_s_ = -1.0;
    MapQualityRunStats run_stats_;
};

inline void write_map_quality_header(std::ofstream &o) {
    o << "timestamp_s,quality_score,quality_level,tof_samples_total,tof_valid_hits,tof_low_confidence,tof_invalid,tof_rejected,tof_valid_ratio,tof_reject_ratio,tof_confidence_mean,front_samples,front_valid_ratio,front_confidence_mean,left_samples,left_valid_ratio,left_confidence_mean,right_samples,right_valid_ratio,right_confidence_mean,map_update_attempts,map_updates,map_update_rate_hz,occupied_cells,free_cells,unknown_cells,known_cells,occupied_ratio,free_ratio,known_ratio,map_alloc_failures,active_scan_recommended,degraded,reason\n";
}

inline void write_map_quality_row(std::ofstream &o, const MapQualitySnapshot &s) {
    o << s.timestamp_s << "," << s.quality_score << "," << s.quality_level << ","
      << s.tof_samples_total << "," << s.tof_valid_hits << "," << s.tof_low_confidence << ","
      << s.tof_invalid << "," << s.tof_rejected << "," << s.tof_valid_ratio << ","
      << s.tof_reject_ratio << "," << s.tof_confidence_mean << ","
      << s.front.samples << "," << map_quality_route_valid_ratio(s.front) << "," << map_quality_route_confidence_mean(s.front) << ","
      << s.left.samples << "," << map_quality_route_valid_ratio(s.left) << "," << map_quality_route_confidence_mean(s.left) << ","
      << s.right.samples << "," << map_quality_route_valid_ratio(s.right) << "," << map_quality_route_confidence_mean(s.right) << ","
      << s.map_update_attempts << "," << s.map_updates << "," << s.map_update_rate_hz << ","
      << s.occupied_cells << "," << s.free_cells << "," << s.unknown_cells << "," << s.known_cells << ","
      << s.occupied_ratio << "," << s.free_ratio << "," << s.known_ratio << ","
      << s.map_alloc_failures << "," << (s.active_scan_recommended ? 1 : 0) << ","
      << (s.degraded ? 1 : 0) << "," << s.reason << "\n";
}

} // namespace robot_slamd
