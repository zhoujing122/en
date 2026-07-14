#include "sparse_map_artifact_test_helpers.hpp"

#include <cstdio>

int main() {
    using namespace robot_slamd;
    using namespace sparse_map_artifact_test;
    const std::string path = "/tmp/en_m3d3a_sparse_map_roundtrip.map";
    std::remove(path.c_str());
    auto source = artifact();
    std::string reason;
    expect(save_sparse_map_artifact_atomic(path, source, limits(), reason),
           "artifact saves atomically");
    const auto loaded = load_sparse_map_artifact(path, limits());
    expect(loaded.ok, "artifact loads");
    expect(sparse_map_artifact_payload(loaded.artifact) ==
               sparse_map_artifact_payload(source),
           "artifact metadata and evidence round trip exactly");
    expect(sparse_map_fnv1a64(sparse_map_artifact_payload(source)) ==
               sparse_map_fnv1a64(sparse_map_artifact_payload(loaded.artifact)),
           "serialization checksum deterministic");
    std::remove(path.c_str());
    return 0;
}
