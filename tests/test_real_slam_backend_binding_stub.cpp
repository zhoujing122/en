#include "robot_slamd/autonomy/real_adapters/real_slam_backend_binding_stub.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

int main() {
    using namespace robot_slamd;

    RealSlamBackendBindingStub backend;
    expect(!backend.ready(), "default not ready");

    SlamBackendInputFrame input;
    const auto update = backend.update(input);
    expect(update.status == SlamBackendUpdateStatus::Rejected,
           "update rejected");
    expect(update.fault == SlamBackendFault::BackendNotReady,
           "backend not ready fault");
    expect(update.message.find("real_slam_backend_binding_stub_not_implemented") !=
               std::string::npos,
           "stable update message");
    expect(backend.update_call_count() == 1, "update count");

    const auto quality = backend.latest_quality(100.0);
    expect(!quality.good_enough, "quality poor");
    expect(quality.reason == "real_slam_backend_binding_stub_not_implemented",
           "quality reason");

    const auto save = backend.save_map("/tmp/m3b0_stub_map");
    expect(!save.ok, "save fails");
    expect(save.error == "real_slam_backend_binding_stub_not_implemented",
           "save error");
    expect(backend.status().find("waiting_for_real_slam_backend_implementation") !=
               std::string::npos,
           "status names implementation wait");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
