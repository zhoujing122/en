#pragma once

#include "robot_slamd/mapping/sparse_tof/planar_tof_extrinsics.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_grid_types.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_grid_ray_traversal.hpp"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

constexpr std::uint32_t kSparseMapArtifactFormatVersion = 1;

struct SparseMapArtifact {
    std::uint32_t format_version = kSparseMapArtifactFormatVersion;
    std::string map_id;
    std::string run_id;
    std::uint64_t map_revision = 0;
    SparseOccupancyGridConfig grid;
    PlanarThreeTofExtrinsics tof_extrinsics;
    double wheel_base_m = 0.0;
    double wheel_radius_left_m = 0.0;
    double wheel_radius_right_m = 0.0;
    int encoder_ticks_per_rev = 0;
    std::vector<SparseOccupancyCell> cells;
};

struct SparseMapArtifactLimits {
    std::size_t maximum_cells = 100000;
    std::size_t maximum_file_bytes = 16U * 1024U * 1024U;
};

struct SparseMapArtifactResult {
    bool ok = false;
    SparseMapArtifact artifact;
    std::string reason;
};

struct SparseMapArtifactSaveOptions {
    bool fail_before_rename_for_test = false;
};

inline std::uint64_t sparse_map_fnv1a64(const std::string &text) {
    std::uint64_t hash = UINT64_C(14695981039346656037);
    for (unsigned char byte : text) {
        hash ^= static_cast<std::uint64_t>(byte);
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

inline bool sparse_map_artifact_valid(
    const SparseMapArtifact &artifact,
    const SparseMapArtifactLimits &limits,
    std::string &reason) {
    if (artifact.format_version != kSparseMapArtifactFormatVersion) {
        reason = "sparse_map_unknown_format_version";
        return false;
    }
    if (artifact.map_id.empty() || artifact.run_id.empty() ||
        !sparse_grid_config_valid(artifact.grid) ||
        !planar_tof_extrinsic_valid(artifact.tof_extrinsics.front) ||
        !planar_tof_extrinsic_valid(artifact.tof_extrinsics.left) ||
        !planar_tof_extrinsic_valid(artifact.tof_extrinsics.right) ||
        !std::isfinite(artifact.wheel_base_m) ||
        !std::isfinite(artifact.wheel_radius_left_m) ||
        !std::isfinite(artifact.wheel_radius_right_m) ||
        artifact.wheel_base_m <= 0.0 ||
        artifact.wheel_radius_left_m <= 0.0 ||
        artifact.wheel_radius_right_m <= 0.0 ||
        artifact.encoder_ticks_per_rev <= 0) {
        reason = "sparse_map_metadata_invalid";
        return false;
    }
    if (limits.maximum_cells == 0 ||
        artifact.cells.size() > limits.maximum_cells ||
        artifact.cells.size() > artifact.grid.maximum_active_cells) {
        reason = "sparse_map_cell_capacity_exceeded";
        return false;
    }
    for (std::size_t i = 0; i < artifact.cells.size(); ++i) {
        const auto &cell = artifact.cells[i];
        if (cell.evidence < artifact.grid.minimum_evidence ||
            cell.evidence > artifact.grid.maximum_evidence) {
            reason = "sparse_map_cell_evidence_invalid";
            return false;
        }
        if (i > 0 && !(artifact.cells[i - 1].key < cell.key)) {
            reason = "sparse_map_cell_order_or_duplicate_invalid";
            return false;
        }
    }
    reason = "sparse_map_artifact_valid";
    return true;
}

inline std::string sparse_map_artifact_payload(
    const SparseMapArtifact &artifact) {
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "magic ROBOT_SLAMD_SPARSE_MAP\n"
        << "format_version " << artifact.format_version << "\n"
        << "map_id " << std::quoted(artifact.map_id) << "\n"
        << "run_id " << std::quoted(artifact.run_id) << "\n"
        << "map_revision " << artifact.map_revision << "\n"
        << "resolution_m " << artifact.grid.resolution_m << "\n"
        << "free_delta " << artifact.grid.free_delta << "\n"
        << "occupied_delta " << artifact.grid.occupied_delta << "\n"
        << "minimum_evidence " << artifact.grid.minimum_evidence << "\n"
        << "maximum_evidence " << artifact.grid.maximum_evidence << "\n"
        << "free_threshold " << artifact.grid.free_threshold << "\n"
        << "occupied_threshold " << artifact.grid.occupied_threshold << "\n"
        << "maximum_active_cells " << artifact.grid.maximum_active_cells << "\n"
        << "minimum_mapping_range_m " << artifact.grid.minimum_mapping_range_m << "\n"
        << "maximum_mapping_range_m " << artifact.grid.maximum_mapping_range_m << "\n"
        << "no_return_free_space_range_m " << artifact.grid.no_return_free_space_range_m << "\n"
        << "maximum_ray_cells " << artifact.grid.maximum_ray_cells << "\n"
        << "front_extrinsic " << artifact.tof_extrinsics.front.x_m << " "
        << artifact.tof_extrinsics.front.y_m << " "
        << artifact.tof_extrinsics.front.yaw_rad << "\n"
        << "left_extrinsic " << artifact.tof_extrinsics.left.x_m << " "
        << artifact.tof_extrinsics.left.y_m << " "
        << artifact.tof_extrinsics.left.yaw_rad << "\n"
        << "right_extrinsic " << artifact.tof_extrinsics.right.x_m << " "
        << artifact.tof_extrinsics.right.y_m << " "
        << artifact.tof_extrinsics.right.yaw_rad << "\n"
        << "wheel_base_m " << artifact.wheel_base_m << "\n"
        << "wheel_radius_left_m " << artifact.wheel_radius_left_m << "\n"
        << "wheel_radius_right_m " << artifact.wheel_radius_right_m << "\n"
        << "encoder_ticks_per_rev " << artifact.encoder_ticks_per_rev << "\n"
        << "cell_count " << artifact.cells.size() << "\n";
    for (const auto &cell : artifact.cells) {
        out << "cell " << cell.key.x << " " << cell.key.y << " "
            << cell.evidence << "\n";
    }
    return out.str();
}

template <typename T>
inline bool sparse_map_read_field(std::istream &in, const char *expected,
                                  T &value) {
    std::string key;
    return static_cast<bool>(in >> key >> value) && key == expected;
}

inline SparseMapArtifactResult sparse_map_parse_artifact_text(
    const std::string &text, const SparseMapArtifactLimits &limits) {
    SparseMapArtifactResult result;
    if (text.size() > limits.maximum_file_bytes) {
        result.reason = "sparse_map_file_too_large";
        return result;
    }
    const std::string marker = "checksum ";
    const auto checksum_pos = text.rfind(marker);
    if (checksum_pos == std::string::npos ||
        checksum_pos == 0 || text.back() != '\n') {
        result.reason = "sparse_map_checksum_missing";
        return result;
    }
    const std::string payload = text.substr(0, checksum_pos);
    std::istringstream checksum_stream(text.substr(checksum_pos + marker.size()));
    std::uint64_t stored_checksum = 0;
    checksum_stream >> std::hex >> stored_checksum;
    std::string checksum_extra;
    if (!checksum_stream || (checksum_stream >> checksum_extra) ||
        stored_checksum != sparse_map_fnv1a64(payload)) {
        result.reason = "sparse_map_checksum_mismatch";
        return result;
    }

    SparseMapArtifact artifact;
    std::istringstream in(payload);
    std::string key;
    std::string magic;
    if (!(in >> key >> magic) || key != "magic" ||
        magic != "ROBOT_SLAMD_SPARSE_MAP" ||
        !sparse_map_read_field(in, "format_version", artifact.format_version) ||
        !(in >> key >> std::quoted(artifact.map_id)) || key != "map_id" ||
        !(in >> key >> std::quoted(artifact.run_id)) || key != "run_id" ||
        !sparse_map_read_field(in, "map_revision", artifact.map_revision) ||
        !sparse_map_read_field(in, "resolution_m", artifact.grid.resolution_m) ||
        !sparse_map_read_field(in, "free_delta", artifact.grid.free_delta) ||
        !sparse_map_read_field(in, "occupied_delta", artifact.grid.occupied_delta) ||
        !sparse_map_read_field(in, "minimum_evidence", artifact.grid.minimum_evidence) ||
        !sparse_map_read_field(in, "maximum_evidence", artifact.grid.maximum_evidence) ||
        !sparse_map_read_field(in, "free_threshold", artifact.grid.free_threshold) ||
        !sparse_map_read_field(in, "occupied_threshold", artifact.grid.occupied_threshold) ||
        !sparse_map_read_field(in, "maximum_active_cells", artifact.grid.maximum_active_cells) ||
        !sparse_map_read_field(in, "minimum_mapping_range_m", artifact.grid.minimum_mapping_range_m) ||
        !sparse_map_read_field(in, "maximum_mapping_range_m", artifact.grid.maximum_mapping_range_m) ||
        !sparse_map_read_field(in, "no_return_free_space_range_m", artifact.grid.no_return_free_space_range_m) ||
        !sparse_map_read_field(in, "maximum_ray_cells", artifact.grid.maximum_ray_cells)) {
        result.reason = "sparse_map_header_invalid";
        return result;
    }
    auto read_extrinsic = [&in](const char *name, PlanarTofExtrinsic &out) {
        std::string field;
        return static_cast<bool>(in >> field >> out.x_m >> out.y_m >> out.yaw_rad) &&
               field == name;
    };
    std::size_t cell_count = 0;
    if (!read_extrinsic("front_extrinsic", artifact.tof_extrinsics.front) ||
        !read_extrinsic("left_extrinsic", artifact.tof_extrinsics.left) ||
        !read_extrinsic("right_extrinsic", artifact.tof_extrinsics.right) ||
        !sparse_map_read_field(in, "wheel_base_m", artifact.wheel_base_m) ||
        !sparse_map_read_field(in, "wheel_radius_left_m", artifact.wheel_radius_left_m) ||
        !sparse_map_read_field(in, "wheel_radius_right_m", artifact.wheel_radius_right_m) ||
        !sparse_map_read_field(in, "encoder_ticks_per_rev", artifact.encoder_ticks_per_rev) ||
        !sparse_map_read_field(in, "cell_count", cell_count) ||
        cell_count > limits.maximum_cells) {
        result.reason = "sparse_map_metadata_parse_failed";
        return result;
    }
    artifact.cells.reserve(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
        SparseOccupancyCell cell;
        int evidence = 0;
        if (!(in >> key >> cell.key.x >> cell.key.y >> evidence) ||
            key != "cell" ||
            evidence < std::numeric_limits<std::int16_t>::min() ||
            evidence > std::numeric_limits<std::int16_t>::max()) {
            result.reason = "sparse_map_cell_parse_failed";
            return result;
        }
        cell.evidence = static_cast<std::int16_t>(evidence);
        artifact.cells.push_back(cell);
    }
    std::string extra;
    if (in >> extra) {
        result.reason = "sparse_map_trailing_payload";
        return result;
    }
    if (!sparse_map_artifact_valid(artifact, limits, result.reason)) {
        return result;
    }
    result.ok = true;
    result.artifact = std::move(artifact);
    result.reason = "sparse_map_artifact_loaded";
    return result;
}

inline bool sparse_map_grid_config_equal(const SparseOccupancyGridConfig &a,
                                         const SparseOccupancyGridConfig &b) {
    return a.resolution_m == b.resolution_m &&
           a.free_delta == b.free_delta &&
           a.occupied_delta == b.occupied_delta &&
           a.minimum_evidence == b.minimum_evidence &&
           a.maximum_evidence == b.maximum_evidence &&
           a.free_threshold == b.free_threshold &&
           a.occupied_threshold == b.occupied_threshold &&
           a.maximum_active_cells == b.maximum_active_cells &&
           a.minimum_mapping_range_m == b.minimum_mapping_range_m &&
           a.maximum_mapping_range_m == b.maximum_mapping_range_m &&
           a.no_return_free_space_range_m == b.no_return_free_space_range_m &&
           a.maximum_ray_cells == b.maximum_ray_cells;
}

inline bool sparse_map_extrinsics_equal(const PlanarThreeTofExtrinsics &a,
                                        const PlanarThreeTofExtrinsics &b) {
    const auto equal = [](const PlanarTofExtrinsic &x,
                          const PlanarTofExtrinsic &y) {
        return x.x_m == y.x_m && x.y_m == y.y_m && x.yaw_rad == y.yaw_rad;
    };
    return equal(a.front, b.front) && equal(a.left, b.left) &&
           equal(a.right, b.right);
}

inline bool sparse_map_artifact_compatible(
    const SparseMapArtifact &artifact,
    const SparseOccupancyGridConfig &grid,
    const PlanarThreeTofExtrinsics &extrinsics,
    double wheel_base_m, double wheel_radius_left_m,
    double wheel_radius_right_m, int encoder_ticks_per_rev,
    std::string &reason) {
    if (!sparse_map_grid_config_equal(artifact.grid, grid)) {
        reason = "sparse_map_grid_config_incompatible";
        return false;
    }
    if (!sparse_map_extrinsics_equal(artifact.tof_extrinsics, extrinsics)) {
        reason = "sparse_map_tof_extrinsics_incompatible";
        return false;
    }
    if (artifact.wheel_base_m != wheel_base_m ||
        artifact.wheel_radius_left_m != wheel_radius_left_m ||
        artifact.wheel_radius_right_m != wheel_radius_right_m ||
        artifact.encoder_ticks_per_rev != encoder_ticks_per_rev) {
        reason = "sparse_map_odometry_geometry_incompatible";
        return false;
    }
    reason = "sparse_map_compatible";
    return true;
}

inline SparseMapArtifactResult load_sparse_map_artifact(
    const std::string &path, const SparseMapArtifactLimits &limits) {
    SparseMapArtifactResult result;
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) {
        result.reason = "sparse_map_open_failed";
        return result;
    }
    const auto end = in.tellg();
    if (end < 0 || static_cast<std::uint64_t>(end) >
                       static_cast<std::uint64_t>(limits.maximum_file_bytes)) {
        result.reason = "sparse_map_file_too_large";
        return result;
    }
    std::string text(static_cast<std::size_t>(end), '\0');
    in.seekg(0);
    if (!text.empty() && !in.read(&text[0], static_cast<std::streamsize>(text.size()))) {
        result.reason = "sparse_map_read_failed";
        return result;
    }
    return sparse_map_parse_artifact_text(text, limits);
}

inline bool save_sparse_map_artifact_atomic(
    const std::string &path, const SparseMapArtifact &artifact,
    const SparseMapArtifactLimits &limits, std::string &reason,
    SparseMapArtifactSaveOptions options = {}) {
    if (path.empty() ||
        !sparse_map_artifact_valid(artifact, limits, reason)) {
        if (path.empty()) reason = "sparse_map_path_missing";
        return false;
    }
    const std::string payload = sparse_map_artifact_payload(artifact);
    std::ostringstream checksum;
    checksum << "checksum " << std::hex << std::setfill('0') <<
        std::setw(16) << sparse_map_fnv1a64(payload) << "\n";
    const std::string text = payload + checksum.str();
    if (text.size() > limits.maximum_file_bytes) {
        reason = "sparse_map_file_too_large";
        return false;
    }
    const std::string temporary = path + ".tmp";
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        if (!out || !out.write(text.data(), static_cast<std::streamsize>(text.size()))) {
            reason = "sparse_map_temporary_write_failed";
            std::remove(temporary.c_str());
            return false;
        }
        out.flush();
        if (!out) {
            reason = "sparse_map_temporary_flush_failed";
            std::remove(temporary.c_str());
            return false;
        }
    }
    const auto verified = load_sparse_map_artifact(temporary, limits);
    if (!verified.ok || options.fail_before_rename_for_test) {
        reason = options.fail_before_rename_for_test
                     ? "sparse_map_injected_rename_failure"
                     : verified.reason;
        std::remove(temporary.c_str());
        return false;
    }
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        reason = "sparse_map_atomic_rename_failed";
        std::remove(temporary.c_str());
        return false;
    }
    reason = "sparse_map_saved_atomically";
    return true;
}

} // namespace robot_slamd
