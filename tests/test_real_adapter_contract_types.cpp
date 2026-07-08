#include "robot_slamd/autonomy/contracts/real_adapter_contract_types.hpp"

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

    expect(to_string(RealAdapterKind::Tof) == "tof", "tof string");
    expect(to_string(RealAdapterSeverity::Error) == "error", "error severity string");
    expect(to_string(RealAdapterReadiness::ShadowReady) == "shadow_ready", "shadow ready string");
    expect(real_adapter_kind_id(RealAdapterKind::Tof) == 0, "tof id stable");
    expect(real_adapter_severity_id(RealAdapterSeverity::Error) == 2, "error id stable");
    expect(real_adapter_readiness_id(RealAdapterReadiness::ShadowReady) == 1, "readiness id stable");

    RealAdapterContractReport report;
    expect(!report.ok, "default report not ok");
    expect(report.readiness == RealAdapterReadiness::NotReady, "default readiness not ready");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
