#pragma once
#include "common.hpp"

namespace robot_slamd {

class CsvReader {
public:
    explicit CsvReader(const std::string &path) : path_(path), in_(path) {}
    bool opened() const { return in_.is_open(); }
    bool eof() const { return eof_; }
    const std::string &path() const { return path_; }
    bool next_line(std::vector<std::string> &cols) {
        if (!in_) { eof_ = true; return false; }
        std::string line;
        while (std::getline(in_, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            if (line.find("timestamp") != std::string::npos) continue;
            cols.clear();
            std::stringstream ss(line);
            std::string c;
            while (std::getline(ss, c, ',')) cols.push_back(trim(c));
            return true;
        }
        eof_ = true;
        return false;
    }
private:
    std::string path_;
    std::ifstream in_;
    bool eof_ = false;
};


struct Icm43600FfiConfig {
    uint64_t device_no = 0;
    uint64_t accel_rate_hz = 0;
    uint64_t gyro_rate_hz = 0;
    uint64_t batch_timeout_ms = 0;
    uint32_t accel_fsr = 2;
    uint32_t gyro_fsr = 3;
    bool high_res_fifo = false;
    uint32_t battery_level = 1;
    uint32_t speed_gear = 1;
    double distance_speed_m_s = 0.0;
    bool use_distance_speed_override = false;
};

struct Icm43600Vec3 { double x = 0.0, y = 0.0, z = 0.0; };

class Icm43600FfiClient {
public:
    ~Icm43600FfiClient() { close(); }

    bool init(const Config &cfg, std::string &err) {
        lib_ = dlopen(cfg.imu_icm43600_lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!lib_) { err = std::string("ICM43600 FFI load failed: ") + dlerror(); return false; }
        if (!load_symbols(err)) { close(); return false; }

        Icm43600FfiConfig icfg {};
        if (default_config_) default_config_(&icfg);
        icfg.device_no = cfg.imu_icm43600_device_no;
        icfg.accel_rate_hz = cfg.imu_icm43600_accel_rate_hz;
        icfg.gyro_rate_hz = cfg.imu_icm43600_gyro_rate_hz;
        icfg.batch_timeout_ms = cfg.imu_icm43600_batch_timeout_ms;
        icfg.accel_fsr = cfg.imu_icm43600_accel_fsr;
        icfg.gyro_fsr = cfg.imu_icm43600_gyro_fsr;
        icfg.high_res_fifo = cfg.imu_icm43600_high_res_fifo;
        handle_ = create_(icfg);
        if (!handle_) { err = "ICM43600 create failed"; close(); return false; }
        int rc = start_(handle_);
        if (rc != 0) { err = error_message("ICM43600 start failed", rc); close(); return false; }
        return true;
    }

    int read_gyro(Icm43600Vec3 &gyro_rad_s, std::string &err) {
        if (!handle_) { err = "ICM43600 handle is not initialized"; return -2; }
        int rc = update_(handle_);
        if (rc == 1) return 1;
        if (rc != 0) { err = error_message("ICM43600 update failed", rc); return rc; }
        rc = get_angular_velocity_(handle_, &gyro_rad_s);
        if (rc != 0) { err = error_message("ICM43600 get angular velocity failed", rc); return rc; }
        return 0;
    }

private:
    using DefaultConfigFn = int (*)(Icm43600FfiConfig *);
    using CreateFn = void *(*)(Icm43600FfiConfig);
    using DestroyFn = void (*)(void *);
    using StartStopFn = int (*)(void *);
    using UpdateFn = int (*)(void *);
    using GetAngularVelocityFn = int (*)(void *, Icm43600Vec3 *);
    using LastErrorFn = const char *(*)(const void *);

    template <typename Fn>
    bool load_symbol(Fn &fn, const char *name, std::string &err) {
        dlerror();
        fn = reinterpret_cast<Fn>(dlsym(lib_, name));
        const char *e = dlerror();
        if (e || !fn) { err = std::string("ICM43600 missing symbol ") + name + ": " + (e ? e : "null"); return false; }
        return true;
    }

    bool load_symbols(std::string &err) {
        return load_symbol(default_config_, "icm43600_default_config", err) &&
               load_symbol(create_, "icm43600_create", err) &&
               load_symbol(destroy_, "icm43600_destroy", err) &&
               load_symbol(start_, "icm43600_start", err) &&
               load_symbol(stop_, "icm43600_stop", err) &&
               load_symbol(update_, "icm43600_update", err) &&
               load_symbol(get_angular_velocity_, "icm43600_get_angular_velocity", err) &&
               load_symbol(last_error_, "icm43600_last_error", err);
    }

    std::string error_message(const char *prefix, int rc) const {
        std::ostringstream o;
        o << prefix << ": rc=" << rc;
        if (last_error_ && handle_) {
            const char *msg = last_error_(handle_);
            if (msg && *msg) o << ": " << msg;
        }
        return o.str();
    }

    void close() {
        if (handle_) {
            if (stop_) stop_(handle_);
            if (destroy_) destroy_(handle_);
            handle_ = nullptr;
        }
        if (lib_) { dlclose(lib_); lib_ = nullptr; }
    }

    void *lib_ = nullptr;
    void *handle_ = nullptr;
    DefaultConfigFn default_config_ = nullptr;
    CreateFn create_ = nullptr;
    DestroyFn destroy_ = nullptr;
    StartStopFn start_ = nullptr;
    StartStopFn stop_ = nullptr;
    UpdateFn update_ = nullptr;
    GetAngularVelocityFn get_angular_velocity_ = nullptr;
    LastErrorFn last_error_ = nullptr;
};


namespace cjc_bl4820 {

inline uint8_t checksum(const std::vector<uint8_t> &bytes, size_t begin, size_t end) {
    uint32_t sum = 0;
    for (size_t i = begin; i < end; ++i) sum += bytes[i];
    return static_cast<uint8_t>((~sum) & 0xff);
}

inline uint16_t le_u16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline int16_t le_i16(const uint8_t *p) {
    return static_cast<int16_t>(le_u16(p));
}

inline double current_raw_to_a(uint16_t raw) {
    return static_cast<double>(raw) * 0.1;
}

inline double pair_skew_us(double left_timestamp_s, double right_timestamp_s) {
    if (!std::isfinite(left_timestamp_s) || !std::isfinite(right_timestamp_s)) return std::numeric_limits<double>::infinity();
    return std::fabs(left_timestamp_s - right_timestamp_s) * 1000000.0;
}

inline bool pair_skew_ok(double left_timestamp_s, double right_timestamp_s, double max_pair_skew_us) {
    return pair_skew_us(left_timestamp_s, right_timestamp_s) <= max_pair_skew_us;
}

inline bool parse_read_payload(const std::vector<uint8_t> &payload, CjcBl4820WheelReading &out, std::string &reason) {
    out = CjcBl4820WheelReading{};
    if (payload.size() != 6) { reason = "bad_payload_length"; return false; }
    out.position_raw = le_u16(&payload[0]);
    out.rpm = le_i16(&payload[2]);
    out.current_raw = le_u16(&payload[4]);
    out.current_a = current_raw_to_a(out.current_raw);
    out.ok = true;
    out.reason = "ok";
    reason = "ok";
    return true;
}

inline void record_parse_failure(EncoderStats &stats, bool is_left, const std::string &reason, bool checksum_error) {
    if (is_left) {
        stats.left_parse_error_count++;
        if (checksum_error) stats.left_checksum_error_count++;
        if (reason.find("status") != std::string::npos) stats.left_status_error_count++;
    } else {
        stats.right_parse_error_count++;
        if (checksum_error) stats.right_checksum_error_count++;
        if (reason.find("status") != std::string::npos) stats.right_status_error_count++;
    }
}

inline void record_timeout(EncoderStats &stats, bool is_left) {
    if (is_left) { stats.left_timeout = true; stats.left_timeout_count++; }
    else { stats.right_timeout = true; stats.right_timeout_count++; }
}

inline std::vector<uint8_t> read_request(uint8_t id) {
    std::vector<uint8_t> req{0xff, 0xff, id, 0x04, 0x02, 0x24, 0x06, 0x00};
    req.back() = checksum(req, 2, req.size() - 1);
    return req;
}

inline int64_t unwrap_delta(uint16_t last_pos, uint16_t current_pos, int counts_per_rev) {
    int64_t delta = static_cast<int64_t>(current_pos) - static_cast<int64_t>(last_pos);
    const int64_t half = counts_per_rev / 2;
    if (delta > half) delta -= counts_per_rev;
    else if (delta < -half) delta += counts_per_rev;
    return delta;
}

inline bool parse_read_response(const std::vector<uint8_t> &frame, uint8_t expected_id,
                                CjcBl4820WheelReading &out, std::string &reason,
                                bool &checksum_error) {
    checksum_error = false;
    out = CjcBl4820WheelReading{};
    if (frame.size() < 4) { reason = "short_frame"; return false; }
    if (frame[0] != 0xff || frame[1] != 0xff) { reason = "bad_header"; return false; }
    if (frame[2] != expected_id) { reason = "unexpected_id"; return false; }
    const size_t total = static_cast<size_t>(frame[3]) + 4;
    if (frame.size() != total) { reason = "bad_length"; return false; }
    if (checksum(frame, 2, frame.size() - 1) != frame.back()) {
        checksum_error = true;
        reason = "checksum_error";
        return false;
    }
    if (frame[3] != 0x0b) { reason = "unexpected_read_length"; return false; }
    out.status = frame[4];
    if (out.status != 0x00) { reason = "status_error"; return false; }
    if (frame[5] != 0x04 || frame[6] != 0x02 || frame[7] != 0x24) {
        reason = "bad_read_echo";
        return false;
    }
    std::vector<uint8_t> payload(frame.begin() + 8, frame.begin() + 14);
    if (!parse_read_payload(payload, out, reason)) return false;
    out.status = frame[4];
    return true;
}

} // namespace cjc_bl4820

class CjcBl4820UartEncoder {
public:
    explicit CjcBl4820UartEncoder(const Config &cfg) : cfg_(cfg) {
        left_.name = "left";
        left_.path = cfg_.encoder_left_port;
        left_.id = static_cast<uint8_t>(cfg_.encoder_left_id);
        left_.sign = cfg_.encoder_left_sign;
        left_.is_left = true;
        right_.name = "right";
        right_.path = cfg_.encoder_right_port;
        right_.id = static_cast<uint8_t>(cfg_.encoder_right_id);
        right_.sign = cfg_.encoder_right_sign;
        right_.is_left = false;
    }

    ~CjcBl4820UartEncoder() { close_port(left_); close_port(right_); }

    bool init(std::string &err) {
        if (!open_port(left_, err)) return false;
        if (!open_port(right_, err)) { close_port(left_); return false; }
        return true;
    }

    bool read_sample(uint64_t t_us, EncoderSample &sample) {
        last_log_ = EncoderLogSample{};
        last_log_.t_us = t_us;
        last_log_.decision = "hold_last";
        last_log_.left_total_ticks = left_.state.total_ticks;
        last_log_.right_total_ticks = right_.state.total_ticks;

        CjcBl4820WheelReading left_read, right_read;
        std::string left_reason, right_reason;
        bool left_ok = read_wheel(left_, left_read, left_reason);
        bool right_ok = read_wheel(right_, right_read, right_reason);
        update_pair_skew();
        fill_log_from_readings(left_read, right_read);

        if (!left_ok || !right_ok) {
            last_log_.reason = join_nonempty(left_reason, right_reason);
            mark_unhealthy_from_consecutive_errors();
            return false;
        }

        int64_t left_delta = 0, right_delta = 0, left_total = 0, right_total = 0;
        if (!candidate_total(left_, left_read.position_raw, left_delta, left_total, left_reason) ||
            !candidate_total(right_, right_read.position_raw, right_delta, right_total, right_reason)) {
            stats_.jump_rejects++;
            last_log_.decision = "rejected";
            last_log_.reason = join_nonempty(left_reason, right_reason);
            mark_error(left_);
            mark_error(right_);
            mark_unhealthy_from_consecutive_errors();
            return false;
        }

        commit(left_, left_read.position_raw, left_delta, left_total, t_us);
        commit(right_, right_read.position_raw, right_delta, right_total, t_us);
        stats_.left_total_ticks = left_.state.total_ticks;
        stats_.right_total_ticks = right_.state.total_ticks;
        stats_.left_rpm = left_read.rpm;
        stats_.right_rpm = right_read.rpm;
        stats_.left_speed_rpm = left_read.rpm;
        stats_.right_speed_rpm = right_read.rpm;
        stats_.left_current_a = left_read.current_a;
        stats_.right_current_a = right_read.current_a;
        stats_.left_status = left_read.status;
        stats_.right_status = right_read.status;

        last_log_.decision = "accepted";
        last_log_.reason = "ok";
        last_log_.left_delta_ticks = left_delta;
        last_log_.right_delta_ticks = right_delta;
        last_log_.left_total_ticks = left_.state.total_ticks;
        last_log_.right_total_ticks = right_.state.total_ticks;

        last_sample_ = {t_us, left_.state.total_ticks, right_.state.total_ticks};
        have_last_sample_ = true;
        sample = last_sample_;
        return true;
    }

    bool have_last_sample() const { return have_last_sample_; }
    EncoderSample last_sample(uint64_t t_us) const {
        EncoderSample out = last_sample_;
        out.t_us = t_us;
        return out;
    }
    const EncoderStats &stats() const { return stats_; }
    const EncoderLogSample &last_log() const { return last_log_; }

private:
    struct WheelState {
        bool initialized = false;
        uint16_t last_pos = 0;
        int64_t total_ticks = 0;
        uint64_t last_t_us = 0;
    };

    struct WheelPort {
        std::string name;
        std::string path;
        uint8_t id = 1;
        int sign = 1;
        bool is_left = true;
        int fd = -1;
        WheelState state;
    };

    static speed_t baud_constant(int baudrate) {
#ifdef B1000000
        if (baudrate == 1000000) return B1000000;
#endif
        (void)baudrate;
        return 0;
    }

    bool open_port(WheelPort &port, std::string &err) {
        port.fd = open(port.path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (port.fd < 0) {
            err = "CJC BL4820 " + port.name + " UART open failed: " + port.path + ": " + std::strerror(errno);
            return false;
        }
        speed_t speed = baud_constant(cfg_.encoder_baudrate);
        if (speed == 0) {
            err = "CJC BL4820 unsupported UART baudrate: " + std::to_string(cfg_.encoder_baudrate);
            close_port(port);
            return false;
        }
        termios tio {};
        if (tcgetattr(port.fd, &tio) < 0) {
            err = "CJC BL4820 tcgetattr failed on " + port.path + ": " + std::strerror(errno);
            close_port(port);
            return false;
        }
        cfmakeraw(&tio);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
#ifdef CRTSCTS
        tio.c_cflag &= ~CRTSCTS;
#endif
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (cfsetispeed(&tio, speed) < 0 || cfsetospeed(&tio, speed) < 0) {
            err = "CJC BL4820 cfset speed failed on " + port.path + ": " + std::strerror(errno);
            close_port(port);
            return false;
        }
        if (tcsetattr(port.fd, TCSANOW, &tio) < 0) {
            err = "CJC BL4820 tcsetattr failed on " + port.path + ": " + std::strerror(errno);
            close_port(port);
            return false;
        }
        tcflush(port.fd, TCIOFLUSH);
        return true;
    }

    bool read_wheel(WheelPort &port, CjcBl4820WheelReading &reading, std::string &reason) {
        begin_read_attempt(port);
        const uint64_t start_us = now_us_steady();
        if (port.fd < 0) {
            reason = port.name + "_uart_not_open";
            finish_read_attempt(port, start_us, false);
            record_error(port, reason);
            return false;
        }
        std::vector<uint8_t> req = cjc_bl4820::read_request(port.id);
        tcflush(port.fd, TCIFLUSH);
        if (!write_all(port.fd, req, reason)) {
            reason = port.name + "_" + reason;
            finish_read_attempt(port, start_us, false);
            record_error(port, reason);
            return false;
        }
        std::vector<uint8_t> frame;
        bool checksum_seen = false;
        if (!read_frame(port.fd, port.id, frame, reason, checksum_seen)) {
            const std::string raw_reason = reason;
            reason = port.name + "_" + reason;
            finish_read_attempt(port, start_us, false);
            if (raw_reason.find("timeout") != std::string::npos) cjc_bl4820::record_timeout(stats_, port.is_left);
            record_error(port, reason);
            if (checksum_seen) { stats_.uart_checksum_errors++; record_checksum_error(port); }
            return false;
        }
        if (checksum_seen) { stats_.uart_checksum_errors++; record_checksum_error(port); }
        bool checksum_error = false;
        if (!cjc_bl4820::parse_read_response(frame, port.id, reading, reason, checksum_error)) {
            const std::string raw_reason = reason;
            reason = port.name + "_" + reason;
            finish_read_attempt(port, start_us, false);
            cjc_bl4820::record_parse_failure(stats_, port.is_left, raw_reason, checksum_error);
            if (checksum_error) stats_.uart_checksum_errors++;
            else if (reading.status != 0xff) stats_.uart_status_errors++;
            else stats_.uart_frame_errors++;
            record_error(port, reason);
            return false;
        }
        finish_read_attempt(port, start_us, true);
        record_success(port);
        record_reading(port, reading);
        return true;
    }

    bool write_all(int fd, const std::vector<uint8_t> &data, std::string &reason) const {
        size_t off = 0;
        while (off < data.size()) {
            ssize_t n = ::write(fd, data.data() + off, data.size() - off);
            if (n > 0) { off += static_cast<size_t>(n); continue; }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd pfd{fd, POLLOUT, 0};
                int rc = poll(&pfd, 1, cfg_.encoder_uart_timeout_ms);
                if (rc > 0) continue;
                reason = rc == 0 ? "write_timeout" : "write_poll_failed";
                return false;
            }
            reason = "write_failed";
            return false;
        }
        return true;
    }

    bool read_frame(int fd, uint8_t expected_id, std::vector<uint8_t> &frame,
                    std::string &reason, bool &checksum_seen) const {
        checksum_seen = false;
        std::vector<uint8_t> buf;
        const uint64_t deadline = now_us_steady() + static_cast<uint64_t>(cfg_.encoder_uart_timeout_ms) * 1000;
        while (now_us_steady() < deadline) {
            uint64_t now = now_us_steady();
            int remain_ms = now >= deadline ? 1 : static_cast<int>((deadline - now + 999) / 1000);
            if (remain_ms < 1) remain_ms = 1;
            pollfd pfd{fd, POLLIN, 0};
            int rc = poll(&pfd, 1, remain_ms);
            if (rc < 0) {
                if (errno == EINTR) continue;
                reason = "read_poll_failed";
                return false;
            }
            if (rc == 0) continue;
            uint8_t tmp[64];
            ssize_t n = ::read(fd, tmp, sizeof(tmp));
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
                reason = "read_failed";
                return false;
            }
            if (n == 0) continue;
            buf.insert(buf.end(), tmp, tmp + n);
            while (buf.size() >= 4) {
                auto it = std::search(buf.begin(), buf.end(), kHeader_.begin(), kHeader_.end());
                if (it == buf.end()) { buf.clear(); break; }
                if (it != buf.begin()) buf.erase(buf.begin(), it);
                if (buf.size() < 4) break;
                size_t total = static_cast<size_t>(buf[3]) + 4;
                if (total < 6 || total > 64) { buf.erase(buf.begin()); continue; }
                if (buf.size() < total) break;
                std::vector<uint8_t> candidate(buf.begin(), buf.begin() + total);
                buf.erase(buf.begin(), buf.begin() + total);
                if (candidate[2] != expected_id) continue;
                if (cjc_bl4820::checksum(candidate, 2, candidate.size() - 1) != candidate.back()) {
                    checksum_seen = true;
                    continue;
                }
                frame.swap(candidate);
                return true;
            }
        }
        reason = checksum_seen ? "checksum_error" : "read_timeout";
        return false;
    }

    bool candidate_total(const WheelPort &port, uint16_t raw_pos, int64_t &signed_delta,
                         int64_t &candidate_total_ticks, std::string &reason) const {
        if (raw_pos > static_cast<uint16_t>(cfg_.encoder_ticks_per_rev)) {
            reason = port.name + "_position_out_of_range";
            return false;
        }
        uint16_t pos = static_cast<uint16_t>(raw_pos % cfg_.encoder_ticks_per_rev);
        if (!port.state.initialized) {
            signed_delta = 0;
            candidate_total_ticks = 0;
            reason = "ok";
            return true;
        }
        int64_t delta = cjc_bl4820::unwrap_delta(port.state.last_pos, pos, cfg_.encoder_ticks_per_rev);
        if (std::llabs(delta) >= cfg_.encoder_ticks_per_rev / 2) {
            reason = port.name + "_ambiguous_position_jump";
            return false;
        }
        signed_delta = static_cast<int64_t>(port.sign) * delta;
        candidate_total_ticks = port.state.total_ticks + signed_delta;
        reason = "ok";
        return true;
    }

    void commit(WheelPort &port, uint16_t raw_pos, int64_t, int64_t total_ticks, uint64_t t_us) {
        port.state.initialized = true;
        port.state.last_pos = static_cast<uint16_t>(raw_pos % cfg_.encoder_ticks_per_rev);
        port.state.total_ticks = total_ticks;
        port.state.last_t_us = t_us;
    }

    void update_pair_skew() {
        stats_.encoder_pair_skew_us = cjc_bl4820::pair_skew_us(stats_.left_timestamp_s, stats_.right_timestamp_s);
        stats_.encoder_pair_skew_ok = cjc_bl4820::pair_skew_ok(stats_.left_timestamp_s, stats_.right_timestamp_s, cfg_.encoder_max_pair_skew_us);
        if (!stats_.encoder_pair_skew_ok) stats_.encoder_pair_skew_bad_count++;
    }

    void begin_read_attempt(WheelPort &port) {
        if (port.is_left) {
            stats_.left_read_ok = false; stats_.left_frame_ok = false; stats_.left_checksum_ok = false; stats_.left_timeout = false;
        } else {
            stats_.right_read_ok = false; stats_.right_frame_ok = false; stats_.right_checksum_ok = false; stats_.right_timeout = false;
        }
    }

    void finish_read_attempt(WheelPort &port, uint64_t start_us, bool ok) {
        const uint64_t end_us = now_us_steady();
        const double latency = static_cast<double>(end_us - start_us);
        const double timestamp_s = static_cast<double>(end_us) / 1000000.0;
        if (port.is_left) {
            stats_.left_latency_us = latency; stats_.left_timestamp_s = timestamp_s; stats_.left_read_ok = ok;
            if (ok) { stats_.left_frame_ok = true; stats_.left_checksum_ok = true; }
        } else {
            stats_.right_latency_us = latency; stats_.right_timestamp_s = timestamp_s; stats_.right_read_ok = ok;
            if (ok) { stats_.right_frame_ok = true; stats_.right_checksum_ok = true; }
        }
    }

    void record_checksum_error(WheelPort &port) {
        if (port.is_left) stats_.left_checksum_error_count++;
        else stats_.right_checksum_error_count++;
    }

    void record_reading(WheelPort &port, const CjcBl4820WheelReading &reading) {
        if (port.is_left) {
            stats_.left_speed_rpm = reading.rpm;
            stats_.left_current_a = reading.current_a;
            stats_.left_status = reading.status;
        } else {
            stats_.right_speed_rpm = reading.rpm;
            stats_.right_current_a = reading.current_a;
            stats_.right_status = reading.status;
        }
    }

    void fill_log_from_readings(const CjcBl4820WheelReading &left_read, const CjcBl4820WheelReading &right_read) {
        last_log_.left_pos_raw = left_read.position_raw;
        last_log_.right_pos_raw = right_read.position_raw;
        last_log_.left_rpm = left_read.rpm;
        last_log_.right_rpm = right_read.rpm;
        last_log_.left_current_raw = left_read.current_raw;
        last_log_.right_current_raw = right_read.current_raw;
        last_log_.left_status = left_read.status;
        last_log_.right_status = right_read.status;
    }

    void record_success(WheelPort &port) {
        if (port.is_left) {
            stats_.left_reads++;
            stats_.left_consecutive_errors = 0;
            stats_.left_unhealthy = false;
        } else {
            stats_.right_reads++;
            stats_.right_consecutive_errors = 0;
            stats_.right_unhealthy = false;
        }
    }

    void record_error(WheelPort &port, const std::string &reason) {
        if (reason.find("timeout") != std::string::npos) stats_.uart_timeout_errors++;
        if (reason.find("frame") != std::string::npos || reason.find("header") != std::string::npos || reason.find("length") != std::string::npos || reason.find("echo") != std::string::npos) stats_.uart_frame_errors++;
        mark_error(port);
    }

    void mark_error(WheelPort &port) {
        if (port.is_left) {
            stats_.left_errors++;
            stats_.left_consecutive_errors++;
        } else {
            stats_.right_errors++;
            stats_.right_consecutive_errors++;
        }
    }

    void mark_unhealthy_from_consecutive_errors() {
        stats_.left_unhealthy = stats_.left_consecutive_errors >= static_cast<uint64_t>(cfg_.encoder_consecutive_error_limit);
        stats_.right_unhealthy = stats_.right_consecutive_errors >= static_cast<uint64_t>(cfg_.encoder_consecutive_error_limit);
    }

    static std::string join_nonempty(const std::string &left, const std::string &right) {
        if (left.empty()) return right;
        if (right.empty()) return left;
        if (left == "ok") return right;
        if (right == "ok") return left;
        return left + "|" + right;
    }

    void close_port(WheelPort &port) {
        if (port.fd >= 0) {
            close(port.fd);
            port.fd = -1;
        }
    }

    const Config &cfg_;
    WheelPort left_;
    WheelPort right_;
    EncoderStats stats_;
    bool have_last_sample_ = false;
    EncoderSample last_sample_;
    EncoderLogSample last_log_;
    static constexpr std::array<uint8_t, 2> kHeader_{{0xff, 0xff}};
};

struct MT3801MeasurementData { int32_t Obj_Range; uint8_t RangeStatus; uint8_t Confidence; };
#define MT3801_IOCTL_POWER_ON _IO('p', 0x06)
#define MT3801_IOCTL_INIT _IO('p', 0x07)
#define MT3801_IOCTL_CONTINUOUS_START _IO('p', 0x08)
#define MT3801_IOCTL_CONTINUOUS_STOP _IO('p', 0x09)
#define MT3801_IOCTL_GET_DATAS _IOR('p', 0x0a, MT3801MeasurementData)
#define MT3801_IOCTL_POWER_OFF _IO('p', 0x0b)

class SensorManager {
public:
    explicit SensorManager(const Config &cfg) : cfg_(cfg), enc_csv_(cfg.encoder_path), imu_csv_(cfg.imu_path), tof_csv_(cfg.tof_csv_path), cjc_bl4820_encoder_(cfg) {}
    ~SensorManager() {
        for (auto &kv : mt_fds_) {
            ioctl(kv.second, MT3801_IOCTL_CONTINUOUS_STOP, nullptr);
            ioctl(kv.second, MT3801_IOCTL_POWER_OFF, nullptr);
            close(kv.second);
        }
    }
    bool init(std::string &err) {
        if (cfg_.encoder_source == "csv" && !enc_csv_.opened()) { err = "encoder CSV not found: " + cfg_.encoder_path; return false; }
        if (cfg_.encoder_source == "cjc_bl4820_uart" && !cjc_bl4820_encoder_.init(err)) return false;
        if (cfg_.imu_source == "csv" && !imu_csv_.opened()) { err = "imu CSV not found: " + cfg_.imu_path; return false; }
        if (cfg_.imu_source == "icm43600_ffi" && !imu_icm43600_.init(cfg_, err)) return false;
        if (cfg_.tof_source == "csv" && !tof_csv_.opened()) { err = "tof CSV not found: " + cfg_.tof_csv_path; return false; }
        if (cfg_.tof_source != "mt3801_ioctl") return true;
        for (auto &id : {"front", "left", "right"}) {
            std::string path = cfg_.tof_devices.at(id);
            if (path.empty()) continue;
            int fd = open(path.c_str(), O_RDWR | O_SYNC);
            if (fd < 0) { err = "MT3801 device not found: " + path; return false; }
            if (ioctl(fd, MT3801_IOCTL_POWER_ON, nullptr) < 0 || ioctl(fd, MT3801_IOCTL_INIT, nullptr) < 0 ||
                ioctl(fd, MT3801_IOCTL_CONTINUOUS_START, nullptr) < 0) {
                err = "MT3801 init failed: " + path + ": " + std::strerror(errno);
                ioctl(fd, MT3801_IOCTL_CONTINUOUS_STOP, nullptr);
                ioctl(fd, MT3801_IOCTL_POWER_OFF, nullptr);
                close(fd);
                return false;
            }
            mt_fds_[id] = fd;
        }
        return true;
    }
    EncoderSample read_encoder(uint64_t t_us, double sim_t) {
        if (cfg_.encoder_source == "csv") {
            std::vector<std::string> c;
            if (enc_csv_.next_line(c) && c.size() >= 3) {
                last_encoder_csv_ = {static_cast<uint64_t>(std::strtoull(c[0].c_str(), nullptr, 10)), std::strtoll(c[1].c_str(), nullptr, 10), std::strtoll(c[2].c_str(), nullptr, 10)};
                have_encoder_csv_ = true;
                return last_encoder_csv_;
            }
            add_warning("encoder_eof");
            add_warning("encoder_gap");
            if (have_encoder_csv_) return last_encoder_csv_;
            return {t_us, 0, 0};
        }
        if (cfg_.encoder_source == "cjc_bl4820_uart") {
            EncoderSample enc;
            if (cjc_bl4820_encoder_.read_sample(t_us, enc)) return enc;
            add_warning("encoder_gap");
            const auto &elog = cjc_bl4820_encoder_.last_log();
            if (elog.reason.find("checksum") != std::string::npos) add_warning("encoder_uart_checksum_error");
            if (elog.reason.find("timeout") != std::string::npos) add_warning("encoder_uart_timeout");
            if (elog.reason.find("status") != std::string::npos) add_warning("encoder_uart_status_error");
            if (elog.reason.find("jump") != std::string::npos) add_warning("encoder_jump");
            const auto &st = cjc_bl4820_encoder_.stats();
            if (st.left_unhealthy) add_warning("encoder_left_unhealthy");
            if (st.right_unhealthy) add_warning("encoder_right_unhealthy");
            if (cjc_bl4820_encoder_.have_last_sample()) return cjc_bl4820_encoder_.last_sample(t_us);
            return {t_us, 0, 0};
        }
        double v = 0.12, w = 0.10;
        double vl = v - w * cfg_.wheel_base_m * 0.5;
        double vr = v + w * cfg_.wheel_base_m * 0.5;
        double rev_l = (vl * sim_t) / (2.0 * kPi * cfg_.wheel_radius_left_m);
        double rev_r = (vr * sim_t) / (2.0 * kPi * cfg_.wheel_radius_right_m);
        return {t_us, static_cast<int64_t>(rev_l * cfg_.encoder_ticks_per_rev), static_cast<int64_t>(rev_r * cfg_.encoder_ticks_per_rev)};
    }
    ImuSample read_imu(uint64_t t_us, double) {
        if (cfg_.imu_source == "icm43600_ffi") {
            std::string err;
            Icm43600Vec3 gyro {};
            int rc = imu_icm43600_.read_gyro(gyro, err);
            if (rc == 0) { last_imu_csv_ = make_imu_sample(t_us, select_gyro_axis(gyro)); have_imu_csv_ = true; return last_imu_csv_; }
            add_warning(rc == 1 ? "imu_gap" : "imu_read_fail");
            if (!err.empty()) add_warning(rc == 1 ? "icm43600_no_data" : "icm43600_error");
            if (have_imu_csv_) return last_imu_csv_;
            return make_imu_sample(t_us, 0.0);
        }
        if (cfg_.imu_source == "csv") {
            std::vector<std::string> c;
            if (imu_csv_.next_line(c) && c.size() >= 2) {
                last_imu_csv_ = make_imu_sample(static_cast<uint64_t>(std::strtoull(c[0].c_str(), nullptr, 10)), std::strtod(c[1].c_str(), nullptr));
                have_imu_csv_ = true;
                return last_imu_csv_;
            }
            add_warning("imu_eof");
            add_warning("imu_gap");
            if (have_imu_csv_) return last_imu_csv_;
            return make_imu_sample(t_us, 0.0);
        }
        return make_imu_sample(t_us, 0.10);
    }
    std::vector<TofSample> read_tof(uint64_t t_us, const Pose &pose) {
        tof_events_.clear();
        if (cfg_.tof_source == "csv") {
            std::vector<TofSample> out;
            for (int i = 0; i < 3; ++i) {
                std::vector<std::string> c;
                if (tof_csv_.next_line(c) && c.size() >= 5) out.push_back({static_cast<uint64_t>(std::strtoull(c[0].c_str(), nullptr, 10)), c[1], std::atoi(c[2].c_str()), std::atoi(c[3].c_str()), std::atoi(c[4].c_str())});
            }
            if (!out.empty()) {
                last_tof_csv_ = out;
                have_tof_csv_ = true;
                return out;
            }
            add_warning("tof_eof");
            add_warning("tof_gap");
            tof_events_.push_back({t_us, "all", "tof_eof"});
            if (have_tof_csv_) {
                std::vector<TofSample> held = last_tof_csv_;
                for (auto &smp : held) smp.t_us = t_us;
                return held;
            }
            return {};
        }
        if (cfg_.tof_source == "mt3801_ioctl") {
            std::vector<TofSample> out;
            std::map<std::string, bool> seen{{"front", false}, {"left", false}, {"right", false}};
            for (auto &kv : mt_fds_) {
                MT3801MeasurementData d {};
                if (ioctl(kv.second, MT3801_IOCTL_GET_DATAS, &d) == 0) {
                    out.push_back({t_us, kv.first, d.Obj_Range, d.RangeStatus, d.Confidence});
                    seen[kv.first] = true;
                } else {
                    add_warning("tof_read_fail");
                    tof_events_.push_back({t_us, kv.first, "tof_read_fail"});
                }
            }
            for (auto &id : {"front", "left", "right"}) {
                if (!seen[id]) {
                    std::string tag = std::string("tof_missing_") + id;
                    add_warning(tag);
                    tof_events_.push_back({t_us, id, tag});
                }
            }
            if (out.empty()) {
                consecutive_empty_tof_reads_++;
                add_warning("tof_gap");
                tof_events_.push_back({t_us, "all", "tof_gap"});
            } else {
                consecutive_empty_tof_reads_ = 0;
            }
            return out;
        }
        return sim_tof(t_us, pose);
    }
    std::vector<std::string> consume_warnings() {
        std::vector<std::string> out;
        out.swap(warnings_);
        return out;
    }
    std::vector<TofLogEvent> consume_tof_events() {
        std::vector<TofLogEvent> out;
        out.swap(tof_events_);
        return out;
    }
    const EncoderStats &encoder_stats() const { return cjc_bl4820_encoder_.stats(); }
    const EncoderLogSample &last_encoder_log() const { return cjc_bl4820_encoder_.last_log(); }
private:

    double select_gyro_axis(const Icm43600Vec3 &gyro) const {
        double v = gyro.z;
        if (cfg_.imu_gyro_axis == "x") v = gyro.x;
        else if (cfg_.imu_gyro_axis == "y") v = gyro.y;
        return static_cast<double>(cfg_.imu_gyro_sign) * v;
    }
    ImuSample make_imu_sample(uint64_t t_us, double mapped_gyro_z_rad_s) const {
        double used = (cfg_.imu_mode == "log_only") ? 0.0 : mapped_gyro_z_rad_s;
        return {t_us, used, mapped_gyro_z_rad_s};
    }
    void add_warning(const std::string &w) { add_unique(warnings_, w); }
    std::vector<TofSample> sim_tof(uint64_t t_us, const Pose &pose) {
        std::vector<TofSample> out;
        for (auto &id : {"front", "left", "right"}) {
            const auto &e = cfg_.tof_extrinsics.at(id);
            double cy = std::cos(pose.yaw), sy = std::sin(pose.yaw);
            double sx = pose.x + cy * e.x_m - sy * e.y_m;
            double syw = pose.y + sy * e.x_m + cy * e.y_m;
            double yaw = pose.yaw + deg2rad(e.yaw_deg);
            double r = ray_box(sx, syw, yaw, -2.0, -2.0, 2.0, 2.0);
            int status = (r > cfg_.tof_max_range_m) ? 255 : 0;
            int mm = static_cast<int>(std::min(r, cfg_.tof_max_range_m + 1.5) * 1000.0);
            out.push_back({t_us, id, mm, status, status == 0 ? 85 : 10});
        }
        return out;
    }
    static double ray_box(double x, double y, double yaw, double xmin, double ymin, double xmax, double ymax) {
        double dx = std::cos(yaw), dy = std::sin(yaw);
        double best = 1e9;
        if (std::fabs(dx) > 1e-6) {
            for (double bx : {xmin, xmax}) {
                double t = (bx - x) / dx, yy = y + t * dy;
                if (t > 0 && yy >= ymin && yy <= ymax) best = std::min(best, t);
            }
        }
        if (std::fabs(dy) > 1e-6) {
            for (double by : {ymin, ymax}) {
                double t = (by - y) / dy, xx = x + t * dx;
                if (t > 0 && xx >= xmin && xx <= xmax) best = std::min(best, t);
            }
        }
        return best;
    }
    const Config &cfg_;
    CsvReader enc_csv_, imu_csv_, tof_csv_;
    std::map<std::string, int> mt_fds_;
    Icm43600FfiClient imu_icm43600_;
    CjcBl4820UartEncoder cjc_bl4820_encoder_;
    bool have_encoder_csv_ = false;
    bool have_imu_csv_ = false;
    bool have_tof_csv_ = false;
    EncoderSample last_encoder_csv_;
    ImuSample last_imu_csv_;
    std::vector<TofSample> last_tof_csv_;
    int consecutive_empty_tof_reads_ = 0;
    std::vector<std::string> warnings_;
    std::vector<TofLogEvent> tof_events_;
};

} // namespace robot_slamd
