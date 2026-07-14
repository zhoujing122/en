#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

struct SparseTofKeyframeConfig {
    std::size_t max_keyframes = 64;
    std::size_t max_total_rays = 12288;
};

struct SparseTofKeyframe {
    std::uint64_t keyframe_id = 0;
    std::uint64_t bundle_id = 0;
    std::uint64_t reference_map_revision = 0;
    std::uint64_t committed_map_revision = 0;
    double collection_start_timestamp_s = 0.0;
    double collection_end_timestamp_s = 0.0;
    MapFromOdom2D map_from_odom_before = identity_map_from_odom();
    MapFromOdom2D map_from_odom_after = identity_map_from_odom();
    double matcher_delta_yaw_rad = 0.0;
    double matcher_score = 0.0;
    double matcher_margin = 0.0;
    SparseTofLocalMatchStatus matcher_status =
        SparseTofLocalMatchStatus::NotRun;
    std::size_t snapshot_count = 0;
    std::size_t ray_count = 0;
    std::size_t hit_ray_count = 0;
    std::size_t no_return_ray_count = 0;
    std::size_t changed_cell_count = 0;
    std::shared_ptr<const FrozenMultiTofObservationBundle> frozen_bundle;
};

class PreparedSparseTofKeyframeCommit {
public:
    bool ready() const { return ready_; }
    const SparseTofKeyframe *keyframe() const { return keyframe_.get(); }

private:
    friend class SparseTofKeyframeManager;
    bool ready_ = false;
    std::vector<std::shared_ptr<const SparseTofKeyframe>> staged_keyframes_;
    std::shared_ptr<const SparseTofKeyframe> keyframe_;
    std::size_t staged_total_rays_ = 0;
    std::uint64_t staged_next_keyframe_id_ = 1;
};

struct SparseTofKeyframePrepareResult {
    bool ok = false;
    PreparedSparseTofKeyframeCommit prepared;
    std::string reason;
};

class SparseTofKeyframeManager {
public:
    explicit SparseTofKeyframeManager(SparseTofKeyframeConfig config = {})
        : config_(config) {}

    SparseTofKeyframePrepareResult prepare(SparseTofKeyframe keyframe) const {
        SparseTofKeyframePrepareResult result;
        if (config_.max_keyframes == 0 || config_.max_total_rays == 0 ||
            keyframe.bundle_id == 0 || !keyframe.frozen_bundle ||
            !keyframe.frozen_bundle->available()) {
            result.reason = "keyframe_invalid_request";
            return result;
        }
        for (const auto &existing : keyframes_) {
            if (existing->bundle_id == keyframe.bundle_id) {
                result.reason = "keyframe_bundle_already_committed";
                return result;
            }
        }
        if (keyframes_.size() >= config_.max_keyframes ||
            keyframe.ray_count > config_.max_total_rays - total_rays_) {
            result.reason = "keyframe_capacity_reached";
            return result;
        }

        keyframe.keyframe_id = next_keyframe_id_;
        auto immutable =
            std::make_shared<const SparseTofKeyframe>(std::move(keyframe));
        result.prepared.staged_keyframes_ = keyframes_;
        result.prepared.staged_keyframes_.push_back(immutable);
        result.prepared.keyframe_ = immutable;
        result.prepared.staged_total_rays_ =
            total_rays_ + immutable->ray_count;
        result.prepared.staged_next_keyframe_id_ = next_keyframe_id_ + 1;
        result.prepared.ready_ = true;
        result.ok = true;
        result.reason = "keyframe_prepared";
        return result;
    }

    void commit(PreparedSparseTofKeyframeCommit &&prepared) noexcept {
        keyframes_.swap(prepared.staged_keyframes_);
        total_rays_ = prepared.staged_total_rays_;
        next_keyframe_id_ = prepared.staged_next_keyframe_id_;
        prepared.ready_ = false;
    }

    std::size_t size() const { return keyframes_.size(); }
    std::size_t total_rays() const { return total_rays_; }
    const SparseTofKeyframe *latest() const {
        return keyframes_.empty() ? nullptr : keyframes_.back().get();
    }

private:
    SparseTofKeyframeConfig config_;
    std::vector<std::shared_ptr<const SparseTofKeyframe>> keyframes_;
    std::size_t total_rays_ = 0;
    std::uint64_t next_keyframe_id_ = 1;
};

} // namespace robot_slamd
