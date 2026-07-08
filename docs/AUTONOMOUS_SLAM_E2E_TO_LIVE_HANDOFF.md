# Autonomous SLAM E2E To Live Handoff

Before live transition, RealTofImuWheelSensorPort, RealSoftwareMotionPort, and RealSlamBackendBinding must be implemented. M2-B3 software transport acceptance, M3-A1 real adapter contract, M3-A3 SLAM backend acceptance, and M3-A4 E2E pre-live must pass. Operators must still verify emergency stop, lifted direction probe, STOP/E-STOP/TTL behavior, and low-speed in-place scan. Forward/backward remains disabled until the ground safety gate is complete.
