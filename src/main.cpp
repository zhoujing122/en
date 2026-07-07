#include "robot_slamd/app/app.hpp"

namespace {

void on_signal(int) { robot_slamd::g_stop = 1; }

} // namespace

int main(int argc, char **argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    try {
        return robot_slamd::real_main(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    }
}
