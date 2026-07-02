#pragma once
#include "common.hpp"

namespace robot_slamd {

class TofFilter {
public:
    explicit TofFilter(const Config &c) : cfg_(c) {}
    TofFilterResult process(const TofSample &s) {
        TofFilterResult out;
        double range_m = s.raw_range_mm / 1000.0;
        out.range_m = range_m;
        out.filtered_confidence = clamp_confidence(s.confidence);

        if (s.range_status == 255) {
            out.map_update = true;
            out.hit = false;
            out.range_m = cfg_.tof_max_range_m;
            out.filtered_confidence = 0;
            out.stability_count = ++free_count_[s.id];
            stable_count_[s.id] = 0;
            out.decision = "free_only";
            return out;
        }
        free_count_[s.id] = 0;
        if (s.range_status == 6) { out.decision = "log_only"; stable_count_[s.id] = 0; return out; }
        if (s.range_status != 0) { out.decision = "rejected_status"; stable_count_[s.id] = 0; return out; }
        if (s.confidence <= 0) { out.decision = "invalid_confidence"; stable_count_[s.id] = 0; return out; }
        if (s.confidence < cfg_.confidence_min) { out.decision = "rejected_confidence"; stable_count_[s.id] = 0; return out; }
        if (range_m < cfg_.tof_min_range_m || range_m > cfg_.tof_max_range_m) { out.decision = "rejected_range"; stable_count_[s.id] = 0; return out; }

        auto &hist = range_hist_[s.id];
        auto &conf_hist = confidence_hist_[s.id];
        if (!hist.empty() && std::fabs(range_m - median(hist)) > cfg_.jump_reject_m) {
            out.decision = "rejected_jump";
            stable_count_[s.id] = 0;
            return out;
        }
        if (hist.size() >= 3) {
            double med = median(hist);
            double mad = median_abs_dev(hist, med);
            double gate = std::max(cfg_.mad_reject_m, 3.0 * mad);
            if (std::fabs(range_m - med) > gate) {
                out.decision = "rejected_mad";
                stable_count_[s.id] = 0;
                return out;
            }
        }

        hist.push_back(range_m);
        conf_hist.push_back(clamp_confidence(s.confidence));
        int win = std::max(1, cfg_.median_window);
        if (static_cast<int>(hist.size()) > win) hist.erase(hist.begin());
        if (static_cast<int>(conf_hist.size()) > win) conf_hist.erase(conf_hist.begin());
        double med = median(hist);
        int conf = median_int(conf_hist);
        out.range_m = med;
        out.filtered_confidence = conf;

        auto last_it = last_filtered_.find(s.id);
        if (last_it != last_filtered_.end() && std::fabs(med - last_it->second) <= std::max(cfg_.mad_reject_m, cfg_.resolution_m)) {
            stable_count_[s.id]++;
        } else {
            stable_count_[s.id] = 1;
        }
        last_filtered_[s.id] = med;
        out.stability_count = stable_count_[s.id];
        if (stable_count_[s.id] < std::max(1, cfg_.stable_hits_required)) {
            out.decision = "warming_up";
            return out;
        }

        out.map_update = true;
        out.hit = true;
        out.decision = "hit_update";
        return out;
    }
private:
    static int clamp_confidence(int c) { return std::max(0, std::min(255, c)); }
    static double median(std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }
    static int median_int(std::vector<int> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }
    static double median_abs_dev(const std::vector<double> &v, double med) {
        std::vector<double> d;
        d.reserve(v.size());
        for (double x : v) d.push_back(std::fabs(x - med));
        return median(d);
    }
    const Config &cfg_;
    std::map<std::string, std::vector<double>> range_hist_;
    std::map<std::string, std::vector<int>> confidence_hist_;
    std::map<std::string, double> last_filtered_;
    std::map<std::string, int> stable_count_;
    std::map<std::string, int> free_count_;
};

struct TofHealthStats {
    uint64_t samples = 0;
    uint64_t hits = 0;
    uint64_t rejects = 0;
    uint64_t gaps = 0;
    uint64_t confidence_sum = 0;
    uint64_t confidence_count = 0;

    void record_sample(const TofFilterResult &res) {
        samples++;
        if (res.map_update && res.hit) hits++;
        else if (!res.map_update) rejects++;
        if (res.hit || !res.map_update) {
            confidence_sum += static_cast<uint64_t>(std::max(0, res.filtered_confidence));
            confidence_count++;
        }
    }
    void record_gap() { gaps++; }
    double hit_rate() const { return samples ? static_cast<double>(hits) / samples : 0.0; }
    double reject_rate() const { return samples ? static_cast<double>(rejects) / samples : 0.0; }
    double confidence_mean() const { return confidence_count ? static_cast<double>(confidence_sum) / confidence_count : 0.0; }
    bool unhealthy(const Config &cfg) const {
        uint64_t total = samples + gaps;
        if (total < 20) return false;
        if (gaps * 2 > total) return true;
        if (samples >= 20 && reject_rate() > 0.80) return true;
        if (confidence_count >= 20 && confidence_mean() < cfg.confidence_min) return true;
        return false;
    }
};

struct TofHealth {
    std::map<std::string, TofHealthStats> by_id{{"front", {}}, {"left", {}}, {"right", {}}};
    void record_sample(const std::string &id, const TofFilterResult &res) { by_id[id].record_sample(res); }
    void record_gap(const std::string &id) { by_id[id].record_gap(); }
    bool unhealthy(const std::string &id, const Config &cfg) const {
        auto it = by_id.find(id);
        return it != by_id.end() && it->second.unhealthy(cfg);
    }
};

} // namespace robot_slamd
