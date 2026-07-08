#include "robot_slamd/autonomy/map_backend/slam_backend_types.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool contains(const std::vector<std::string> &values, const std::string &needle) {
    for (const auto &value : values) {
        if (value == needle) {
            return true;
        }
    }
    return false;
}
} // namespace

int main() {
    using namespace robot_slamd;

    expect(to_string(SlamBackendStage::Idle) == "idle", "stage idle string");
    expect(to_string(SlamBackendFault::MissingTof) == "missing_tof", "fault missing tof string");
    expect(to_string(SlamBackendUpdateStatus::Accepted) == "accepted", "status accepted string");
    expect(slam_backend_stage_id(SlamBackendStage::UpdatingMap) == 3, "stage id stable");
    expect(slam_backend_fault_id(SlamBackendFault::MissingTof) == 1, "fault id stable");
    expect(slam_backend_update_status_id(SlamBackendUpdateStatus::Fault) == 3, "status id stable");

    SlamBackendUpdateResult update;
    expect(update.status == SlamBackendUpdateStatus::Rejected, "default update rejected");
    SlamBackendSaveResult save;
    expect(!save.ok, "default save not ok");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
