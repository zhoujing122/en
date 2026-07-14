#include "sparse_tof_yaw_match_test_helpers.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe.hpp"

int main() {
    using namespace robot_slamd;
    using namespace yaw_match_test;
    const auto frames = asymmetric_frames();
    auto frozen = std::make_shared<const FrozenMultiTofObservationBundle>(
        frozen_bundle(frames, frames.size() * 3));
    SparseTofKeyframeManager manager({1, 64});

    SparseTofKeyframe keyframe;
    keyframe.bundle_id = frozen->summary().bundle_id;
    keyframe.reference_map_revision = 11;
    keyframe.committed_map_revision = 12;
    keyframe.ray_count = frames.size() * 3;
    keyframe.frozen_bundle = frozen;
    auto prepared = manager.prepare(keyframe);
    expect(prepared.ok && prepared.prepared.keyframe()->keyframe_id == 1,
           "keyframe id prepared without consumption");
    expect(manager.size() == 0, "prepare does not change manager");
    manager.commit(std::move(prepared.prepared));
    expect(manager.size() == 1 && manager.latest()->bundle_id == 41,
           "keyframe committed once");

    expect(!manager.prepare(keyframe).ok,
           "same bundle cannot create another keyframe");
    keyframe.bundle_id = 42;
    expect(!manager.prepare(keyframe).ok,
           "bounded keyframe capacity fails closed");
    expect(manager.size() == 1 && manager.total_rays() == frames.size() * 3,
           "failed prepares preserve keyframe state");
    return 0;
}
