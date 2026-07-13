#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_checker.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
void expect_near(double actual, double expected, double eps, const char *message) {
    if (std::fabs(actual - expected) > eps) {
        std::cerr << "FAIL: " << message << " actual=" << actual
                  << " expected=" << expected << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    MultiTofContractChecker checker;
    auto frame = make_multi_tof_frame(MultiTofMountId::Front, "tof_front_frame", 0.0, 100.000, 100.004);
    const auto valid = checker.validate_reading(frame, 100.004);
    expect(valid.protocol_valid, "2731mm protocol valid");
    expect(valid.usable_for_mapping, "2731mm confidence 100 usable");
    expect_near(valid.effective_timestamp_s, 100.002, 1e-12, "midpoint timestamp");

    frame.echo_tag_u48++;
    const auto changed_echo = checker.validate_reading(frame, 100.004);
    expect_near(changed_echo.effective_timestamp_s, 100.002, 1e-12,
                "echo tag does not affect time");

    frame.timing = make_request_timing(100.010, 100.014);
    const auto changed_window = checker.validate_reading(frame, 100.014);
    expect_near(changed_window.effective_timestamp_s, 100.012, 1e-12,
                "same echo different window changes time");

    auto below = frame;
    below.distance_mm = 19;
    expect(!checker.validate_reading(below, 100.014).protocol_valid, "19mm invalid");
    auto at_min = frame;
    at_min.distance_mm = 20;
    auto at_min_reading = checker.validate_reading(at_min, 100.014);
    expect(at_min_reading.protocol_valid && !at_min_reading.usable_for_mapping, "20mm protocol only");
    auto mapping_min = frame;
    mapping_min.distance_mm = 50;
    expect(checker.validate_reading(mapping_min, 100.014).usable_for_mapping, "50mm mapping usable");
    auto at_max = frame;
    at_max.distance_mm = 12000;
    expect(checker.validate_reading(at_max, 100.014).usable_for_mapping, "12000mm valid");
    auto above_max = frame;
    above_max.distance_mm = 12001;
    expect(!checker.validate_reading(above_max, 100.014).protocol_valid, "12001mm invalid");

    auto conf0 = frame;
    conf0.confidence = 0;
    expect(!checker.validate_reading(conf0, 100.014).usable_for_mapping, "confidence 0 unusable");
    auto conf69 = frame;
    conf69.confidence = 69;
    const auto diagnostic = checker.validate_reading(conf69, 100.014);
    expect(diagnostic.protocol_valid && !diagnostic.usable_for_mapping, "confidence 69 diagnostic only");
    auto conf70 = frame;
    conf70.confidence = 70;
    expect(checker.validate_reading(conf70, 100.014).usable_for_mapping, "confidence 70 usable");
    auto conf101 = frame;
    conf101.confidence = 101;
    expect(!checker.validate_reading(conf101, 100.014).protocol_valid, "confidence 101 invalid");

    expect_near(scalar_tof_full_fov_from_spot_width(0.330, 12.0) * 180.0 / kPi,
                1.57553465, 1e-6, "12m 330mm fov");
    expect_near(scalar_tof_full_fov_from_spot_width(0.260, 10.0) * 180.0 / kPi,
                1.48960636, 1e-6, "10m 260mm fov");
    expect_near(scalar_tof_spot_width_from_full_fov(deg2rad(1.6), 12.0),
                0.335125, 1e-6, "1.6deg 12m width");
    expect_near(scalar_tof_spot_width_from_full_fov(deg2rad(1.5), 10.0),
                0.261814, 1e-6, "1.5deg 10m width");
    expect_near(scalar_tof_default_full_fov_rad(), deg2rad(1.6), 1e-12, "default fov 1.6");

    bool threw = false;
    try { scalar_tof_full_fov_from_spot_width(0.0, 12.0); } catch (...) { threw = true; }
    expect(threw, "zero width rejected");
    threw = false;
    try { scalar_tof_full_fov_from_spot_width(0.330, 0.0); } catch (...) { threw = true; }
    expect(threw, "zero distance rejected");
    threw = false;
    try { scalar_tof_spot_width_from_full_fov(std::numeric_limits<double>::quiet_NaN(), 12.0); } catch (...) { threw = true; }
    expect(threw, "nan rejected");
    return failures ? 1 : 0;
}
