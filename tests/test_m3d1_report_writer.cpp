#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

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
}

int main() {
    using namespace robot_slamd;
    SparseMultiTofOccupancyBackendReport report;
    report.native_multi_tof_backend_consumption = true;
    report.legacy_projection_consumed = false;
    report.ground_truth_consumed = false;
    report.hit_ray_count = 3;
    report.no_return_ray_count = 1;
    report.active_cell_count = 12;
    report.map_readiness_score = 0.5;

    const auto text = write_sparse_multi_tof_backend_report(report);
    expect(text.find("sparse three-ToF occupancy backend") != std::string::npos,
           "report names sparse backend");
    expect(text.find("native_multi_tof_backend_consumption=true") !=
               std::string::npos,
           "report states native consumption");
    expect(text.find("legacy_projection_consumed=false") != std::string::npos,
           "report states legacy not consumed");
    expect(text.find("ground_truth_consumed=false") != std::string::npos,
           "report states ground truth not consumed");
    expect(text.find("scan matching not implemented") != std::string::npos,
           "report states scan matching limit");
    expect(text.find("frontier and A* not implemented") != std::string::npos,
           "report states frontier limit");
    expect(text.find("real hardware not accessed") != std::string::npos,
           "report states hardware safety");
    expect(text.find("real slam completed") == std::string::npos,
           "report does not claim real slam");
    return failures ? 1 : 0;
}
