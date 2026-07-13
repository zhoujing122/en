#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sample_data.hpp"

#include <algorithm>
#include <iostream>
#include <string>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
bool contains(const std::vector<std::string> &items, const std::string &needle) {
    return std::find(items.begin(), items.end(), needle) != items.end();
}
}

int main() {
    using namespace robot_slamd;
    MultiTofContractChecker checker;
    const auto result = checker.check_packet(MultiTofSampleData::valid_three_tof_packet(), 100.0);
    expect(result.ok, "valid packet pass");
    expect(result.valid_tof_count == 3, "valid count 3");
    expect(contains(result.passed, "multi_tof_front_protocol_ok"), "front passed");
    expect(contains(result.passed, "multi_tof_left_protocol_ok"), "left passed");
    expect(contains(result.passed, "multi_tof_right_protocol_ok"), "right passed");
    return failures ? 1 : 0;
}
