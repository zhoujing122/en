#pragma once
#include "robot_slamd/core/common.hpp"
#include "robot_slamd/mapping/map_quality.hpp"

namespace robot_slamd {

enum class SupervisorState {
    INIT,
    SENSOR_CHECK,
    MAPPING_READY,
    MAPPING,
    ACTIVE_SCAN_RECOMMENDED,
    DEGRADED,
    LOST,
};

inline const char *supervisor_state_name(SupervisorState s) {
    switch (s) {
    case SupervisorState::INIT: return "INIT";
    case SupervisorState::SENSOR_CHECK: return "SENSOR_CHECK";
    case SupervisorState::MAPPING_READY: return "MAPPING_READY";
    case SupervisorState::MAPPING: return "MAPPING";
    case SupervisorState::ACTIVE_SCAN_RECOMMENDED: return "ACTIVE_SCAN_RECOMMENDED";
    case SupervisorState::DEGRADED: return "DEGRADED";
    case SupervisorState::LOST: return "LOST";
    }
    return "INIT";
}

struct SupervisorSensorInput {
    uint64_t localization_updates = 0;
    uint64_t encoder_left_errors = 0;
    uint64_t encoder_right_errors = 0;
    uint64_t encoder_jump_rejects = 0;
    uint64_t encoder_timeout_errors = 0;
    uint64_t encoder_rejected_updates = 0;
    uint64_t imu_gap_count = 0;
    uint64_t imu_read_fail_count = 0;
    uint64_t gyro_spike_count = 0;
    int unhealthy_tof_routes = 0;
    uint64_t map_alloc_failures = 0;
};

struct MappingSupervisorSnapshot {
    double timestamp_s = 0.0;
    std::string state = "INIT";
    std::string previous_state = "INIT";
    std::string reason = "startup";
    double state_duration_s = 0.0;
    double quality_score = 0.0;
    std::string quality_level = "NO_DATA";
    bool mapping_allowed = true;
    bool active_scan_recommended = false;
    bool degraded = false;
    bool lost = false;
    double tof_valid_ratio = 0.0;
    double tof_reject_ratio = 0.0;
    double map_update_rate_hz = 0.0;
    uint64_t occupied_cells = 0;
    uint64_t known_cells = 0;
    int valid_tof_routes = 0;
    int unhealthy_tof_routes = 0;
    uint64_t encoder_error_count = 0;
    uint64_t imu_error_count = 0;
    uint64_t map_alloc_failures = 0;
};

class MappingSupervisor {
public:
    explicit MappingSupervisor(const Config &cfg) : cfg_(cfg) {}

    bool update(double now_s, const MapQualitySnapshot &map_quality, const SupervisorSensorInput &sensor) {
        if (last_update_s_ < 0.0) {
            state_enter_s_ = now_s;
            last_update_s_ = now_s;
        } else {
            accumulate_state_time(now_s - last_update_s_);
            last_update_s_ = now_s;
        }

        int valid_routes = valid_tof_routes(map_quality);
        bool no_data = map_quality.tof_samples_total == 0 || map_quality.quality_level == "NO_DATA";
        update_condition_timer(no_data, now_s, no_data_since_s_);
        bool low_quality = map_quality.active_scan_recommended ||
                           map_quality.quality_level == "LOW" ||
                           map_quality.quality_score < cfg_.mapping_supervisor_min_good_quality_score ||
                           (cfg_.mapping_supervisor_require_two_tof_routes && valid_routes < 2);
        update_condition_timer(low_quality, now_s, low_quality_since_s_);
        bool low_update = cfg_.map_quality_min_map_update_rate_hz > 0.0 &&
                          map_quality.map_update_rate_hz < cfg_.map_quality_min_map_update_rate_hz;
        update_condition_timer(low_update, now_s, low_update_since_s_);

        uint64_t encoder_errors = encoder_error_count(sensor);
        uint64_t imu_errors = imu_error_count(sensor);
        bool sensor_degraded = threshold_reached(encoder_errors, cfg_.mapping_supervisor_encoder_error_degraded_threshold) ||
                               threshold_reached(imu_errors, cfg_.mapping_supervisor_imu_error_degraded_threshold) ||
                               threshold_reached(sensor.unhealthy_tof_routes, cfg_.mapping_supervisor_tof_unhealthy_degraded_threshold);
        bool no_data_timeout = no_data_since_s_ >= 0.0 && (now_s - no_data_since_s_) >= cfg_.mapping_supervisor_max_no_data_duration_s;
        bool degraded_score = map_quality.quality_score < cfg_.mapping_supervisor_degraded_quality_score && !no_data;
        bool lost_score = map_quality.quality_score <= cfg_.mapping_supervisor_lost_quality_score && !no_data;
        bool map_alloc_failure = sensor.map_alloc_failures > 0 || map_quality.map_alloc_failures > 0;
        bool active_scan_due = (low_quality_since_s_ >= 0.0 && (now_s - low_quality_since_s_) >= cfg_.mapping_supervisor_active_scan_after_low_quality_s) ||
                               (low_update_since_s_ >= 0.0 && (now_s - low_update_since_s_) >= cfg_.mapping_supervisor_active_scan_after_low_update_s);
        bool recovered_good = !no_data &&
                              map_quality.quality_score >= cfg_.mapping_supervisor_min_good_quality_score &&
                              !map_quality.active_scan_recommended &&
                              (!cfg_.mapping_supervisor_require_two_tof_routes || valid_routes >= 2) &&
                              !sensor_degraded && !map_alloc_failure;

        SupervisorState next = state_;
        std::string reason = current_reason(map_quality, sensor, valid_routes, encoder_errors, imu_errors, no_data_timeout, map_alloc_failure, sensor_degraded);
        double state_duration = now_s - state_enter_s_;

        switch (state_) {
        case SupervisorState::INIT:
            reason = "startup";
            if (now_s >= cfg_.mapping_supervisor_startup_grace_s || map_quality.tof_samples_total > 0 || sensor.localization_updates > 0) {
                next = SupervisorState::SENSOR_CHECK;
                reason = "sensor_check";
            }
            break;
        case SupervisorState::SENSOR_CHECK:
            if (no_data_timeout) {
                next = SupervisorState::DEGRADED;
                reason = "no_data_timeout";
            } else if (map_quality.tof_samples_total > 0 || sensor.localization_updates > 0) {
                next = SupervisorState::MAPPING_READY;
                reason = "mapping_ready";
            } else {
                reason = "sensor_check";
            }
            break;
        case SupervisorState::MAPPING_READY:
            if (no_data_timeout || sensor_degraded || map_alloc_failure || degraded_score) {
                next = SupervisorState::DEGRADED;
                reason = current_reason(map_quality, sensor, valid_routes, encoder_errors, imu_errors, no_data_timeout, map_alloc_failure, sensor_degraded);
            } else if (state_duration >= cfg_.mapping_supervisor_min_ready_time_s && recovered_good) {
                next = SupervisorState::MAPPING;
                reason = "mapping_good";
            } else {
                reason = "mapping_ready";
            }
            break;
        case SupervisorState::MAPPING:
            if (lost_score || no_data_timeout) {
                next = SupervisorState::LOST;
                reason = lost_score ? "quality_lost" : "no_data_timeout";
            } else if (sensor_degraded || map_alloc_failure || degraded_score) {
                next = SupervisorState::DEGRADED;
                reason = current_reason(map_quality, sensor, valid_routes, encoder_errors, imu_errors, no_data_timeout, map_alloc_failure, sensor_degraded);
            } else if (active_scan_due) {
                next = SupervisorState::ACTIVE_SCAN_RECOMMENDED;
                reason = active_scan_reason(map_quality, valid_routes);
            } else {
                reason = "mapping_good";
            }
            break;
        case SupervisorState::ACTIVE_SCAN_RECOMMENDED:
            if (lost_score || no_data_timeout) {
                next = SupervisorState::LOST;
                reason = lost_score ? "quality_lost" : "no_data_timeout";
            } else if (sensor_degraded || map_alloc_failure || degraded_score) {
                next = SupervisorState::DEGRADED;
                reason = current_reason(map_quality, sensor, valid_routes, encoder_errors, imu_errors, no_data_timeout, map_alloc_failure, sensor_degraded);
            } else if (recovered_good) {
                next = SupervisorState::MAPPING;
                reason = "recovered_to_mapping";
            } else {
                reason = active_scan_reason(map_quality, valid_routes);
            }
            break;
        case SupervisorState::DEGRADED:
            if (lost_score || no_data_timeout || state_duration >= cfg_.mapping_supervisor_max_degraded_duration_s) {
                next = SupervisorState::LOST;
                reason = state_duration >= cfg_.mapping_supervisor_max_degraded_duration_s ? "degraded_timeout" : (lost_score ? "quality_lost" : "no_data_timeout");
            } else if (recovered_good) {
                next = SupervisorState::MAPPING;
                reason = "recovered_to_mapping";
            } else {
                reason = current_reason(map_quality, sensor, valid_routes, encoder_errors, imu_errors, no_data_timeout, map_alloc_failure, sensor_degraded);
            }
            break;
        case SupervisorState::LOST:
            if (!no_data && map_quality.quality_score > cfg_.mapping_supervisor_lost_quality_score && sensor.localization_updates > 0) {
                next = SupervisorState::SENSOR_CHECK;
                reason = "recovered_to_sensor_check";
            } else {
                reason = "quality_lost";
            }
            break;
        }

        bool changed = next != state_;
        if (changed) {
            previous_state_ = state_;
            state_ = next;
            state_enter_s_ = now_s;
            run_stats_.state_changes++;
            state_changed_since_log_ = true;
        }
        last_reason_ = reason;
        last_snapshot_ = make_snapshot(now_s, map_quality, sensor, valid_routes, encoder_errors, imu_errors);
        return changed;
    }

    bool should_log(double now_s) const {
        if (!cfg_.mapping_supervisor_enabled) return false;
        if (state_changed_since_log_) return true;
        double period_s = 1.0 / cfg_.mapping_supervisor_log_hz;
        if (last_log_s_ < 0.0) return now_s >= period_s;
        return (now_s - last_log_s_) >= period_s;
    }

    MappingSupervisorSnapshot snapshot() const { return last_snapshot_; }

    void mark_logged(double now_s, const MappingSupervisorSnapshot &s) {
        last_log_s_ = now_s;
        state_changed_since_log_ = false;
        run_stats_.state_last = s.state;
        run_stats_.last_reason = s.reason;
        if (s.state == "ACTIVE_SCAN_RECOMMENDED") run_stats_.active_scan_recommended_count++;
        if (s.state == "DEGRADED") run_stats_.degraded_count++;
        if (s.state == "LOST") run_stats_.lost_count++;
    }

    MappingSupervisorRunStats run_stats(double now_s) const {
        MappingSupervisorRunStats out = run_stats_;
        if (last_update_s_ >= 0.0 && now_s > last_update_s_) {
            add_state_time(out, state_, now_s - last_update_s_);
        }
        out.state_last = supervisor_state_name(state_);
        out.last_reason = last_reason_;
        return out;
    }

private:
    void accumulate_state_time(double dt_s) {
        if (dt_s <= 0.0 || !std::isfinite(dt_s)) return;
        add_state_time(run_stats_, state_, dt_s);
    }

    static void add_state_time(MappingSupervisorRunStats &stats, SupervisorState state, double dt_s) {
        if (state == SupervisorState::MAPPING) stats.mapping_seconds += dt_s;
        else if (state == SupervisorState::ACTIVE_SCAN_RECOMMENDED) stats.active_scan_recommended_seconds += dt_s;
        else if (state == SupervisorState::DEGRADED) stats.degraded_seconds += dt_s;
        else if (state == SupervisorState::LOST) stats.lost_seconds += dt_s;
    }

    static void update_condition_timer(bool active, double now_s, double &since_s) {
        if (active) {
            if (since_s < 0.0) since_s = now_s;
        } else {
            since_s = -1.0;
        }
    }

    static int valid_tof_routes(const MapQualitySnapshot &q) {
        int routes = 0;
        if (q.front.valid_hits > 0) routes++;
        if (q.left.valid_hits > 0) routes++;
        if (q.right.valid_hits > 0) routes++;
        return routes;
    }

    static uint64_t encoder_error_count(const SupervisorSensorInput &s) {
        return s.encoder_left_errors + s.encoder_right_errors + s.encoder_jump_rejects + s.encoder_timeout_errors + s.encoder_rejected_updates;
    }

    static uint64_t imu_error_count(const SupervisorSensorInput &s) {
        return s.imu_gap_count + s.imu_read_fail_count + s.gyro_spike_count;
    }

    static bool threshold_reached(uint64_t value, int threshold) {
        return threshold > 0 && value >= static_cast<uint64_t>(threshold);
    }

    static bool threshold_reached(int value, int threshold) {
        return threshold > 0 && value >= threshold;
    }

    std::string active_scan_reason(const MapQualitySnapshot &q, int valid_routes) const {
        if (q.map_update_rate_hz < cfg_.map_quality_min_map_update_rate_hz) return "low_map_update_active_scan";
        if (cfg_.mapping_supervisor_require_two_tof_routes && valid_routes < 2) return "single_tof_route_active_scan";
        if (q.occupied_cells < static_cast<uint64_t>(std::max(0, cfg_.map_quality_min_occupied_cells))) return "too_few_occupied_active_scan";
        return "low_quality_active_scan";
    }

    std::string current_reason(const MapQualitySnapshot &q, const SupervisorSensorInput &s, int valid_routes,
                               uint64_t encoder_errors, uint64_t imu_errors, bool no_data_timeout,
                               bool map_alloc_failure, bool sensor_degraded) const {
        if (no_data_timeout) return "no_data_timeout";
        if (q.tof_samples_total == 0) return "no_tof_data";
        if (map_alloc_failure) return "map_alloc_failure";
        if (threshold_reached(encoder_errors, cfg_.mapping_supervisor_encoder_error_degraded_threshold)) return "encoder_errors";
        if (threshold_reached(imu_errors, cfg_.mapping_supervisor_imu_error_degraded_threshold)) return "imu_errors";
        if (threshold_reached(s.unhealthy_tof_routes, cfg_.mapping_supervisor_tof_unhealthy_degraded_threshold)) return "tof_unhealthy";
        if (q.quality_score <= cfg_.mapping_supervisor_lost_quality_score) return "quality_lost";
        if (q.quality_score < cfg_.mapping_supervisor_degraded_quality_score) return "quality_degraded";
        if (q.tof_valid_ratio < cfg_.map_quality_min_tof_valid_ratio) return "low_tof_valid_ratio";
        if (q.tof_reject_ratio > 0.80) return "high_tof_reject_ratio";
        if (q.map_update_rate_hz < cfg_.map_quality_min_map_update_rate_hz) return "low_map_update_rate";
        if (q.occupied_cells < static_cast<uint64_t>(std::max(0, cfg_.map_quality_min_occupied_cells))) return "too_few_occupied_cells";
        if (cfg_.mapping_supervisor_require_two_tof_routes && valid_routes < 2) return "single_tof_route_only";
        if (sensor_degraded) return "sensor_degraded";
        if (q.active_scan_recommended) return "active_scan_recommended";
        return "mapping_good";
    }

    MappingSupervisorSnapshot make_snapshot(double now_s, const MapQualitySnapshot &q, const SupervisorSensorInput &s,
                                            int valid_routes, uint64_t encoder_errors, uint64_t imu_errors) const {
        MappingSupervisorSnapshot out;
        out.timestamp_s = now_s;
        out.state = supervisor_state_name(state_);
        out.previous_state = supervisor_state_name(previous_state_);
        out.reason = last_reason_;
        out.state_duration_s = now_s - state_enter_s_;
        out.quality_score = q.quality_score;
        out.quality_level = q.quality_level;
        out.mapping_allowed = true;
        out.active_scan_recommended = state_ == SupervisorState::ACTIVE_SCAN_RECOMMENDED || q.active_scan_recommended;
        out.degraded = state_ == SupervisorState::DEGRADED;
        out.lost = state_ == SupervisorState::LOST;
        out.tof_valid_ratio = q.tof_valid_ratio;
        out.tof_reject_ratio = q.tof_reject_ratio;
        out.map_update_rate_hz = q.map_update_rate_hz;
        out.occupied_cells = q.occupied_cells;
        out.known_cells = q.known_cells;
        out.valid_tof_routes = valid_routes;
        out.unhealthy_tof_routes = s.unhealthy_tof_routes;
        out.encoder_error_count = encoder_errors;
        out.imu_error_count = imu_errors;
        out.map_alloc_failures = s.map_alloc_failures > q.map_alloc_failures ? s.map_alloc_failures : q.map_alloc_failures;
        return out;
    }

    const Config &cfg_;
    SupervisorState state_ = SupervisorState::INIT;
    SupervisorState previous_state_ = SupervisorState::INIT;
    double state_enter_s_ = 0.0;
    double last_update_s_ = -1.0;
    double last_log_s_ = -1.0;
    double no_data_since_s_ = -1.0;
    double low_quality_since_s_ = -1.0;
    double low_update_since_s_ = -1.0;
    std::string last_reason_ = "startup";
    bool state_changed_since_log_ = false;
    MappingSupervisorSnapshot last_snapshot_;
    MappingSupervisorRunStats run_stats_;
};

inline void write_supervisor_log_header(std::ofstream &o) {
    o << "timestamp_s,state,previous_state,state_duration_s,reason,quality_score,quality_level,mapping_allowed,active_scan_recommended,degraded,lost,tof_valid_ratio,tof_reject_ratio,map_update_rate_hz,occupied_cells,known_cells,valid_tof_routes,unhealthy_tof_routes,encoder_error_count,imu_error_count,map_alloc_failures\n";
}

inline void write_supervisor_log_row(std::ofstream &o, const MappingSupervisorSnapshot &s) {
    o << s.timestamp_s << "," << s.state << "," << s.previous_state << "," << s.state_duration_s << "," << s.reason << ","
      << s.quality_score << "," << s.quality_level << "," << (s.mapping_allowed ? 1 : 0) << ","
      << (s.active_scan_recommended ? 1 : 0) << "," << (s.degraded ? 1 : 0) << "," << (s.lost ? 1 : 0) << ","
      << s.tof_valid_ratio << "," << s.tof_reject_ratio << "," << s.map_update_rate_hz << ","
      << s.occupied_cells << "," << s.known_cells << "," << s.valid_tof_routes << "," << s.unhealthy_tof_routes << ","
      << s.encoder_error_count << "," << s.imu_error_count << "," << s.map_alloc_failures << "\n";
}

} // namespace robot_slamd
