# Autonomous SLAM E2E To Live Handoff

Before live transition, RealTofImuWheelSensorPort, RealSoftwareMotionPort, and RealSlamBackendBinding must be implemented. M2-B3 software transport acceptance, M3-A1 real adapter contract, M3-A3 SLAM backend acceptance, and M3-A4 E2E pre-live must pass. Operators must still verify emergency stop, lifted direction probe, STOP/E-STOP/TTL behavior, and low-speed in-place scan. Forward/backward remains disabled until the ground safety gate is complete.

## M3-B0 Real Adapter Stubs

M3-B0 adds the fail-closed real adapter stub locations and the live handoff readiness checker. The next live transition still requires replacing the stubs with real sensor, motion, and SLAM backend adapters, then rerunning the M2-B3, M3-A1, M3-A3, and M3-A4 gates.

## M3-B1 Sensor Data Gate

Before live handoff, RealTofImuWheelSensorPort must produce raw packets that pass M3-B1 acceptance. The coordinator remains unchanged because the builder outputs `RobotSlamSensorSnapshot`.

## M3-B2 Replay Gate

Use M3-B2 offline sensor replay before live handoff to validate request-window timing and snapshot conversion with captured data. Replay passing is not live-ready.

## M3-B2.1 Replay Robustness Gate

Before live handoff, offline replay logs should pass the M3-B2.1 robust parser and golden regression harness. This gate improves replay reliability but still does not prove live hardware readiness.
