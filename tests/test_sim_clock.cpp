#include "robot_slamd/simulation/core/sim_clock.hpp"

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
}

int main() {
    using namespace robot_slamd;
    SimClock clock(10.0);
    expect(std::fabs(clock.now_s() - 10.0) < 1e-12, "start time preserved");
    expect(clock.advance(0.02), "advance positive dt");
    expect(std::fabs(clock.now_s() - 10.02) < 1e-12, "advance deterministic");
    expect(!clock.advance(0.0), "reject zero dt");
    expect(!clock.advance(-0.01), "reject negative dt");
    expect(!clock.advance(std::numeric_limits<double>::quiet_NaN()), "reject nan dt");
    expect(!clock.reset(std::numeric_limits<double>::infinity()), "reject infinite reset");

    SimClock a(1.0);
    SimClock b(1.0);
    for (int i = 0; i < 100; ++i) {
        expect(a.advance(0.01), "a advance");
        expect(b.advance(0.01), "b advance");
    }
    expect(std::fabs(a.now_s() - b.now_s()) < 1e-12, "repeatable sequence");
    return failures ? 1 : 0;
}
