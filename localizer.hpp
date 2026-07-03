#pragma once
#include "common.hpp"

namespace robot_slamd {

class Localizer {
public:
    explicit Localizer(const Config &c) : cfg_(c) {}
    void update(const EncoderSample &e, const ImuSample &imu, double dt) {
        warnings_.clear();
        if (!have_) {
            last_ = e;
            have_ = true;
            left_delta_ticks_ = 0;
            right_delta_ticks_ = 0;
            gyro_filtered_ = imu.gyro_z_rad_s;
            have_gyro_filter_ = true;
            return;
        }
        if (!(dt > 0.0) || !std::isfinite(dt)) {
            add_unique(warnings_, "bad_dt");
            return;
        }

        int64_t dl_ticks = e.left_ticks - last_.left_ticks;
        int64_t dr_ticks = e.right_ticks - last_.right_ticks;
        left_delta_ticks_ = dl_ticks;
        right_delta_ticks_ = dr_ticks;
        last_ = e;
        double dl_raw = (static_cast<double>(dl_ticks) / cfg_.encoder_ticks_per_rev) * 2.0 * kPi * cfg_.wheel_radius_left_m;
        double dr_raw = (static_cast<double>(dr_ticks) / cfg_.encoder_ticks_per_rev) * 2.0 * kPi * cfg_.wheel_radius_right_m;
        double max_wheel = std::max(0.01, cfg_.encoder_max_wheel_speed_mps);
        bool encoder_bad = std::fabs(dl_raw / dt) > max_wheel || std::fabs(dr_raw / dt) > max_wheel;
        if (encoder_bad) {
            add_unique(warnings_, "encoder_jump");
            rejected_encoder_updates_++;
            left_delta_ticks_ = 0;
            right_delta_ticks_ = 0;
            dl_m_ = 0.0;
            dr_m_ = 0.0;
        } else {
            dl_m_ = dl_raw;
            dr_m_ = dr_raw;
        }

        double raw_gyro = imu.gyro_z_rad_s;
        if (!have_gyro_filter_) {
            gyro_filtered_ = raw_gyro;
            have_gyro_filter_ = true;
        }
        double proposed = cfg_.gyro_lpf_alpha * raw_gyro + (1.0 - cfg_.gyro_lpf_alpha) * gyro_filtered_;
        if (std::fabs(raw_gyro) > cfg_.gyro_spike_radps || std::fabs(raw_gyro - gyro_filtered_) > cfg_.gyro_spike_radps) {
            add_unique(warnings_, "gyro_spike");
            gyro_spikes_++;
        } else {
            gyro_filtered_ = proposed;
        }

        ds_m_ = 0.5 * (dr_m_ + dl_m_);
        dyaw_enc_ = (dr_m_ - dl_m_) / cfg_.wheel_base_m;
        yaw_enc_ = wrap_pi(yaw_enc_ + dyaw_enc_);
        yaw_imu_ = wrap_pi(yaw_imu_ + (gyro_filtered_ - gyro_bias_) * dt);
        fused_yaw_without_correction_ = wrap_pi(yaw_enc_ + (1.0 - cfg_.yaw_fusion_alpha) * wrap_pi(yaw_imu_ - yaw_enc_));
        pose_.yaw = wrap_pi(fused_yaw_without_correction_ + yaw_correction_offset_rad_);
        pose_.x += ds_m_ * std::cos(pose_.yaw);
        pose_.y += ds_m_ * std::sin(pose_.yaw);
        v_ = ds_m_ / dt;
        w_ = dyaw_enc_ / dt;

        stationary_ = std::fabs(v_) <= cfg_.stationary_linear_speed_mps &&
                      std::fabs(w_) <= cfg_.stationary_angular_speed_radps &&
                      std::fabs(gyro_filtered_ - gyro_bias_) <= std::max(cfg_.static_gyro_max_abs_rad_s, cfg_.stationary_angular_speed_radps);
        if (stationary_ && cfg_.gyro_bias_adapt_alpha > 0.0) {
            gyro_bias_ = (1.0 - cfg_.gyro_bias_adapt_alpha) * gyro_bias_ + cfg_.gyro_bias_adapt_alpha * gyro_filtered_;
            bias_adapt_updates_++;
        }
    }
    void set_gyro_bias(double b) { gyro_bias_ = b; gyro_filtered_ = b; have_gyro_filter_ = true; }
    bool apply_yaw_correction_only(double delta_yaw_rad, const std::string &reason) {
        if (!std::isfinite(delta_yaw_rad)) return false;
        if (std::fabs(delta_yaw_rad) > deg2rad(cfg_.yaw_correction_max_writeback_abs_deg)) return false;
        yaw_correction_offset_rad_ = wrap_pi(yaw_correction_offset_rad_ + delta_yaw_rad);
        pose_.yaw = wrap_pi(pose_.yaw + delta_yaw_rad);
        yaw_correction_apply_count_++;
        yaw_correction_total_abs_deg_ += std::fabs(delta_yaw_rad) * 180.0 / kPi;
        yaw_correction_last_reason_ = reason;
        return true;
    }
    std::vector<std::string> consume_warnings() { std::vector<std::string> out; out.swap(warnings_); return out; }
    const Pose &pose() const { return pose_; }
    double v() const { return v_; }
    double w() const { return w_; }
    double dl() const { return dl_m_; }
    double dr() const { return dr_m_; }
    double ds() const { return ds_m_; }
    double dyaw_enc() const { return dyaw_enc_; }
    double yaw_enc() const { return yaw_enc_; }
    double yaw_imu() const { return yaw_imu_; }
    double yaw_correction_offset_rad() const { return yaw_correction_offset_rad_; }
    uint64_t yaw_correction_apply_count() const { return yaw_correction_apply_count_; }
    double yaw_correction_total_abs_deg() const { return yaw_correction_total_abs_deg_; }
    std::string yaw_correction_last_reason() const { return yaw_correction_last_reason_; }
    double fused_yaw_without_correction_rad() const { return fused_yaw_without_correction_; }
    double gyro_bias() const { return gyro_bias_; }
    double gyro_filtered() const { return gyro_filtered_; }
    int64_t left_delta_ticks() const { return left_delta_ticks_; }
    int64_t right_delta_ticks() const { return right_delta_ticks_; }
    uint64_t rejected_encoder_updates() const { return rejected_encoder_updates_; }
    uint64_t gyro_spikes() const { return gyro_spikes_; }
    uint64_t bias_adapt_updates() const { return bias_adapt_updates_; }
    bool stationary() const { return stationary_; }
    EncoderSample last_encoder() const { return last_; }
private:
    const Config &cfg_;
    bool have_ = false;
    bool have_gyro_filter_ = false;
    bool stationary_ = false;
    EncoderSample last_;
    Pose pose_;
    double yaw_enc_ = 0.0, yaw_imu_ = 0.0, gyro_bias_ = 0.0, gyro_filtered_ = 0.0;
    double fused_yaw_without_correction_ = 0.0, yaw_correction_offset_rad_ = 0.0;
    uint64_t yaw_correction_apply_count_ = 0;
    double yaw_correction_total_abs_deg_ = 0.0;
    std::string yaw_correction_last_reason_;
    double v_ = 0.0, w_ = 0.0, dl_m_ = 0.0, dr_m_ = 0.0, ds_m_ = 0.0, dyaw_enc_ = 0.0;
    int64_t left_delta_ticks_ = 0, right_delta_ticks_ = 0;
    uint64_t rejected_encoder_updates_ = 0;
    uint64_t gyro_spikes_ = 0;
    uint64_t bias_adapt_updates_ = 0;
    std::vector<std::string> warnings_;
};

} // namespace robot_slamd
