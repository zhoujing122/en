#pragma once
#include "common.hpp"

namespace robot_slamd {

enum class GridQueryState {
    UNKNOWN,
    FREE,
    OCCUPIED,
    OUT_OF_MAP,
};

struct GridQueryResult {
    GridQueryState state = GridQueryState::UNKNOWN;
    bool known = false;
    bool occupied = false;
    bool free = false;
};

struct NearestOccupiedResult {
    bool found = false;
    double distance_m = 0.0;
};

struct Chunk {
    std::vector<float> cells;
    std::vector<uint8_t> touched;
    explicit Chunk(int n) : cells(n * n, 0.0f), touched(n * n, 0) {}
};

struct PairHash {
    size_t operator()(const std::pair<int, int> &p) const {
        return (static_cast<uint64_t>(static_cast<uint32_t>(p.first)) << 32) ^ static_cast<uint32_t>(p.second);
    }
};

class ChunkedGrid {
public:
    explicit ChunkedGrid(const Config &c) : cfg_(c) {}

    void add_log_odds_world(double x, double y, double delta) {
        auto cell = cell_for_world(x, y);
        add_log_odds_cell(cell.first, cell.second, delta);
    }

    void update_tof_ray(const Pose &base, const Config::TofExtrinsic &ext, double range_m, bool hit, double free_delta, double occ_delta, const Config &cfg) {
        double cy = std::cos(base.yaw), sy = std::sin(base.yaw);
        double sx = base.x + cy * ext.x_m - sy * ext.y_m;
        double syw = base.y + sy * ext.x_m + cy * ext.y_m;
        double sensor_yaw = base.yaw + deg2rad(ext.yaw_deg);
        double half = deg2rad(ext.fov_deg * 0.5);
        double step = deg2rad(std::max(0.5, cfg.ray_step_deg));
        double free_end = hit ? std::max(0.0, range_m - cfg.hit_thickness_m) : cfg.tof_max_range_m;
        auto start_cell = cell_for_world(sx, syw);
        std::unordered_set<std::pair<int, int>, PairHash> free_cells;
        std::unordered_set<std::pair<int, int>, PairHash> occ_cells;
        int budget = std::max(1, cfg.max_cells_per_tof_update);

        for (double a = -half; a <= half + 1e-6 && budget > 0; a += step) {
            double yaw = sensor_yaw + a;
            auto free_end_cell = cell_for_world(sx + std::cos(yaw) * free_end, syw + std::sin(yaw) * free_end);
            add_line_cells(start_cell.first, start_cell.second, free_end_cell.first, free_end_cell.second, free_cells, budget);
            if (hit) {
                occ_cells.insert(cell_for_world(sx + std::cos(yaw) * range_m, syw + std::sin(yaw) * range_m));
            }
        }
        for (const auto &cocc : occ_cells) free_cells.erase(cocc);
        for (const auto &cfree : free_cells) add_log_odds_cell(cfree.first, cfree.second, free_delta);
        for (const auto &cocc : occ_cells) add_log_odds_cell(cocc.first, cocc.second, occ_delta);
    }


    double log_odds_cell(int gx, int gy) const {
        int cc = cfg_.chunk_cells;
        int cx = floordiv(gx, cc), cy = floordiv(gy, cc);
        auto it = chunks_.find({cx, cy});
        if (it == chunks_.end()) return 0.0;
        int lx = mod(gx, cc), ly = mod(gy, cc), idx = ly * cc + lx;
        if (!it->second.touched[idx]) return 0.0;
        return it->second.cells[idx];
    }

    double probability_cell(int gx, int gy) const {
        double l = log_odds_cell(gx, gy);
        return 1.0 - 1.0 / (1.0 + std::exp(l));
    }

    bool is_occupied_cell(int gx, int gy, const Config &cfg) const {
        int cc = cfg.chunk_cells;
        int cx = floordiv(gx, cc), cy = floordiv(gy, cc);
        auto it = chunks_.find({cx, cy});
        if (it == chunks_.end()) return false;
        int lx = mod(gx, cc), ly = mod(gy, cc), idx = ly * cc + lx;
        return it->second.touched[idx] && probability_cell(gx, gy) >= cfg.occupied_thresh;
    }

    std::pair<int, int> cell_for_world_public(double x, double y) const {
        return cell_for_world(x, y);
    }

    double ray_cast_expected_range(const Pose &base, const Config::TofExtrinsic &ext, double max_range_m, double step_m, const Config &cfg) const {
        double cy = std::cos(base.yaw), sy = std::sin(base.yaw);
        double sx = base.x + cy * ext.x_m - sy * ext.y_m;
        double syw = base.y + sy * ext.x_m + cy * ext.y_m;
        double yaw = base.yaw + deg2rad(ext.yaw_deg);
        double step = std::max(0.001, step_m);
        for (double r = 0.0; r <= max_range_m; r += step) {
            auto cell = cell_for_world(sx + std::cos(yaw) * r, syw + std::sin(yaw) * r);
            if (is_occupied_cell(cell.first, cell.second, cfg)) return r;
        }
        return max_range_m;
    }

    GridQueryResult query_world(double x_m, double y_m, const Config &cfg) const {
        auto cell = cell_for_world(x_m, y_m);
        int cc = cfg.chunk_cells;
        int cx = floordiv(cell.first, cc), cy = floordiv(cell.second, cc);
        auto it = chunks_.find({cx, cy});
        if (it == chunks_.end()) return {GridQueryState::OUT_OF_MAP, false, false, false};
        int lx = mod(cell.first, cc), ly = mod(cell.second, cc), idx = ly * cc + lx;
        if (!it->second.touched[idx]) return {GridQueryState::UNKNOWN, false, false, false};
        double p = probability_cell(cell.first, cell.second);
        if (p >= cfg.occupied_thresh) return {GridQueryState::OCCUPIED, true, true, false};
        if (p <= cfg.free_thresh) return {GridQueryState::FREE, true, false, true};
        return {GridQueryState::UNKNOWN, false, false, false};
    }

    NearestOccupiedResult nearest_occupied_distance(double x_m, double y_m, double radius_m, const Config &cfg) const {
        NearestOccupiedResult out;
        if (radius_m < 0.0) return out;
        auto center = cell_for_world(x_m, y_m);
        int r_cells = std::max(0, static_cast<int>(std::ceil(radius_m / std::max(1e-9, cfg.resolution_m))));
        double best = radius_m + cfg.resolution_m;
        for (int gy = center.second - r_cells; gy <= center.second + r_cells; ++gy) {
            for (int gx = center.first - r_cells; gx <= center.first + r_cells; ++gx) {
                if (!is_occupied_cell(gx, gy, cfg)) continue;
                double cx = (static_cast<double>(gx) + 0.5) * cfg.resolution_m;
                double cy = (static_cast<double>(gy) + 0.5) * cfg.resolution_m;
                double d = std::hypot(cx - x_m, cy - y_m);
                if (d <= radius_m && d < best) {
                    best = d;
                    out.found = true;
                    out.distance_m = d;
                }
            }
        }
        return out;
    }

    bool save(const std::string &pgm, const std::string &yaml, const Config &cfg) const {
        if (!has_bounds_) return false;
        int pad = cfg.chunk_cells;
        int minx = min_gx_ - pad, maxx = max_gx_ + pad;
        int miny = min_gy_ - pad, maxy = max_gy_ + pad;
        int w = maxx - minx + 1, h = maxy - miny + 1;
        std::ofstream img(pgm, std::ios::binary);
        if (!img) return false;
        img << "P5\n" << w << " " << h << "\n255\n";
        for (int yy = maxy; yy >= miny; --yy) {
            for (int xx = minx; xx <= maxx; ++xx) {
                uint8_t px = pixel_for(xx, yy, cfg);
                img.write(reinterpret_cast<const char *>(&px), 1);
            }
        }
        std::ofstream y(yaml);
        if (!y) return false;
        y << "image: map.pgm\n";
        y << "resolution: " << cfg.resolution_m << "\n";
        y << "origin: [" << minx * cfg.resolution_m << ", " << miny * cfg.resolution_m << ", 0.0]\n";
        y << "occupied_thresh: " << cfg.occupied_thresh << "\n";
        y << "free_thresh: " << cfg.free_thresh << "\n";
        y << "negate: 0\n";
        return true;
    }

    GridStats stats(const Config &cfg) const {
        GridStats st;
        st.chunks = chunks_.size();
        int cc = cfg.chunk_cells;
        st.total_cells = static_cast<uint64_t>(st.chunks) * static_cast<uint64_t>(cc) * static_cast<uint64_t>(cc);
        if (has_bounds_) {
            st.min_gx = min_gx_; st.max_gx = max_gx_;
            st.min_gy = min_gy_; st.max_gy = max_gy_;
            st.width_cells = st.max_gx - st.min_gx + 1;
            st.height_cells = st.max_gy - st.min_gy + 1;
        }
        for (const auto &kv : chunks_) {
            const Chunk &chunk = kv.second;
            for (size_t i = 0; i < chunk.cells.size(); ++i) {
                if (!chunk.touched[i]) {
                    st.unknown_cells++;
                    continue;
                }
                st.touched_cells++;
                double l = chunk.cells[i];
                double p = 1.0 - 1.0 / (1.0 + std::exp(l));
                if (p >= cfg.occupied_thresh) st.occupied_cells++;
                else if (p <= cfg.free_thresh) st.free_cells++;
                else st.unknown_cells++;
            }
        }
        st.known_cells = st.occupied_cells + st.free_cells;
        return st;
    }

private:
    static int floordiv(int a, int b) { int q = a / b, r = a % b; return (r && ((r < 0) != (b < 0))) ? q - 1 : q; }
    static int mod(int a, int b) { int r = a % b; return r < 0 ? r + b : r; }

    std::pair<int, int> cell_for_world(double x, double y) const {
        return {static_cast<int>(std::floor(x / cfg_.resolution_m)), static_cast<int>(std::floor(y / cfg_.resolution_m))};
    }

    static void add_line_cells(int x0, int y0, int x1, int y1, std::unordered_set<std::pair<int, int>, PairHash> &out, int &budget) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (budget > 0) {
            if (out.insert({x0, y0}).second) budget--;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void add_log_odds_cell(int gx, int gy, double delta) {
        int cc = cfg_.chunk_cells;
        int cx = floordiv(gx, cc), cy = floordiv(gy, cc);
        int lx = mod(gx, cc), ly = mod(gy, cc);
        auto key = std::make_pair(cx, cy);
        auto it = chunks_.find(key);
        if (it == chunks_.end()) it = chunks_.emplace(key, Chunk(cc)).first;
        int idx = ly * cc + lx;
        float &v = it->second.cells[idx];
        v = static_cast<float>(std::max(cfg_.log_odds_min, std::min(cfg_.log_odds_max, v + delta)));
        it->second.touched[idx] = 1;
        if (!has_bounds_) {
            min_gx_ = max_gx_ = gx;
            min_gy_ = max_gy_ = gy;
            has_bounds_ = true;
        } else {
            min_gx_ = std::min(min_gx_, gx); max_gx_ = std::max(max_gx_, gx);
            min_gy_ = std::min(min_gy_, gy); max_gy_ = std::max(max_gy_, gy);
        }
    }

    uint8_t pixel_for(int gx, int gy, const Config &cfg) const {
        int cc = cfg.chunk_cells;
        int cx = floordiv(gx, cc), cy = floordiv(gy, cc);
        auto it = chunks_.find({cx, cy});
        if (it == chunks_.end()) return 205;
        int lx = mod(gx, cc), ly = mod(gy, cc), idx = ly * cc + lx;
        if (!it->second.touched[idx]) return 205;
        double l = it->second.cells[idx];
        double p = 1.0 - 1.0 / (1.0 + std::exp(l));
        if (p >= cfg.occupied_thresh) return 0;
        if (p <= cfg.free_thresh) return 254;
        return 205;
    }
    const Config &cfg_;
    std::unordered_map<std::pair<int, int>, Chunk, PairHash> chunks_;
    bool has_bounds_ = false;
    int min_gx_ = 0, max_gx_ = 0, min_gy_ = 0, max_gy_ = 0;
};

} // namespace robot_slamd
