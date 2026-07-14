#include "sparse_map_artifact_test_helpers.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <cstdio>
#include <fstream>

int main() {
    using namespace robot_slamd;
    using namespace sparse_map_artifact_test;
    const std::string path = "/tmp/en_m3d3a_sparse_map_failure.map";
    std::remove(path.c_str());
    auto original = artifact();
    std::string reason;
    expect(save_sparse_map_artifact_atomic(path, original, limits(), reason),
           "initial artifact saved");

    auto replacement = original;
    replacement.map_revision++;
    SparseMapArtifactSaveOptions injected;
    injected.fail_before_rename_for_test = true;
    expect(!save_sparse_map_artifact_atomic(
               path, replacement, limits(), reason, injected),
           "injected pre-rename failure rejects");
    auto retained = load_sparse_map_artifact(path, limits());
    expect(retained.ok && retained.artifact.map_revision == original.map_revision,
           "failed atomic save preserves official artifact");

    {
        std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
        char first = 0;
        file.read(&first, 1);
        first = first == 'x' ? 'y' : 'x';
        file.seekp(0);
        file.write(&first, 1);
    }
    expect(!load_sparse_map_artifact(path, limits()).ok,
           "checksum corruption rejects");

    {
        std::ofstream truncated(path, std::ios::binary | std::ios::trunc);
        truncated << "magic ROBOT_SLAMD_SPARSE_MAP\nformat_version 1\n";
    }
    expect(!load_sparse_map_artifact(path, limits()).ok,
           "truncated artifact rejects");

    auto duplicate = artifact();
    duplicate.cells.push_back(duplicate.cells.back());
    expect(!save_sparse_map_artifact_atomic(
               path, duplicate, limits(), reason),
           "duplicate cell rejects before save");
    {
        const auto payload = sparse_map_artifact_payload(duplicate);
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        file << payload << "checksum " << std::hex << std::setfill('0')
             << std::setw(16) << sparse_map_fnv1a64(payload) << "\n";
    }
    expect(!load_sparse_map_artifact(path, limits()).ok,
           "duplicate cell rejects during strict load");

    auto unknown_version = artifact();
    unknown_version.format_version = 2;
    {
        const auto payload = sparse_map_artifact_payload(unknown_version);
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        file << payload << "checksum " << std::hex << std::setfill('0')
             << std::setw(16) << sparse_map_fnv1a64(payload) << "\n";
    }
    expect(!load_sparse_map_artifact(path, limits()).ok,
           "unknown artifact version rejects");

    auto incompatible = artifact();
    incompatible.grid.resolution_m *= 2.0;
    expect(!sparse_map_artifact_compatible(
               original, incompatible.grid, original.tof_extrinsics,
               original.wheel_base_m, original.wheel_radius_left_m,
               original.wheel_radius_right_m,
               original.encoder_ticks_per_rev, reason),
           "resolution incompatibility rejects");
    auto shifted = original.tof_extrinsics;
    shifted.front.x_m += 0.01;
    expect(!sparse_map_artifact_compatible(
               original, original.grid, shifted, original.wheel_base_m,
               original.wheel_radius_left_m,
               original.wheel_radius_right_m,
               original.encoder_ticks_per_rev, reason),
           "extrinsic incompatibility rejects");

    SparseOccupancyGrid grid;
    expect(grid.replace_cells_transactional(original.cells),
           "valid cells install");
    const auto stable = grid.snapshot().cells();
    auto invalid_cells = original.cells;
    invalid_cells.push_back(invalid_cells.back());
    expect(!grid.replace_cells_transactional(invalid_cells) &&
               grid.snapshot().cells().size() == stable.size(),
           "failed staged install leaves live grid unchanged");
    std::remove(path.c_str());
    return 0;
}
