#include "robot_slamd/runtime/sparse_slam_initialization.hpp"

#include <cmath>
#include <iostream>

namespace {

void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

} // namespace

int main() {
    using namespace robot_slamd;

    MapStartupMode startup_mode;
    InitialPoseMode pose_mode;
    expect(parse_map_startup_mode("create_new", startup_mode),
           "parse create_new");
    expect(startup_mode == MapStartupMode::CreateNewMap, "create_new enum");
    expect(parse_map_startup_mode("load_existing", startup_mode),
           "parse load_existing");
    expect(!parse_map_startup_mode("CreateNew", startup_mode),
           "case-sensitive startup mode");
    expect(parse_initial_pose_mode("startup_zero", pose_mode),
           "parse startup_zero");
    expect(parse_initial_pose_mode("configured_pose", pose_mode),
           "parse configured_pose");
    expect(parse_initial_pose_mode("relocalization", pose_mode),
           "parse relocalization");
    expect(!parse_initial_pose_mode("", pose_mode), "empty pose mode rejects");

    MapOdomFrameState state;
    SparseSlamInitializationRequest request;
    request.map_startup_mode = MapStartupMode::LoadExistingMap;
    request.initial_pose_mode = InitialPoseMode::StartupZero;
    auto missing = state.initialize(request);
    expect(!missing.ok, "existing map without pose rejected");
    expect(missing.status == SparseSlamInitializationStatus::InitialPoseMissing,
           "existing map missing pose status");

    request.initial_pose_mode = InitialPoseMode::ConfiguredPose;
    request.has_configured_map_pose = true;
    request.configured_map_pose = make_map_pose(RobotPose2D{});
    auto load = state.initialize(request);
    expect(!load.ok, "existing map load not implemented");
    expect(load.status ==
               SparseSlamInitializationStatus::ExistingMapLoadNotImplemented,
           "existing map load status");

    request.initial_pose_mode = InitialPoseMode::Relocalization;
    auto relocalize = state.initialize(request);
    expect(!relocalize.ok, "relocalization rejected");
    expect(relocalize.status ==
               SparseSlamInitializationStatus::RelocalizationNotImplemented,
           "relocalization status");

    request.map_startup_mode = MapStartupMode::CreateNewMap;
    auto new_relocalize = state.initialize(request);
    expect(!new_relocalize.ok, "new map relocalization rejected");
    expect(new_relocalize.status ==
               SparseSlamInitializationStatus::RelocalizationNotImplemented,
           "new map relocalization not implemented");

    RobotPose2D bad_pose;
    bad_pose.x_m = std::numeric_limits<double>::infinity();
    request.initial_pose_mode = InitialPoseMode::ConfiguredPose;
    request.has_configured_map_pose = true;
    request.configured_map_pose = make_map_pose(bad_pose);
    auto bad_configured = state.initialize(request);
    expect(!bad_configured.ok, "non-finite configured pose rejected");

    return 0;
}
