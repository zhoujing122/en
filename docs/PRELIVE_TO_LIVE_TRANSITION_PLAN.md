# Pre-live To Live Transition Plan

Before any live movement, the project must still complete these steps:

1. Software side implements RealSoftwareMotionPort.
2. Sensor side implements RealTofImuWheelSensorPort.
3. Mapping side implements RealMapPort.
4. M3-A2 pre-live runner passes with real adapters in shadow mode.
5. M2-B3 software transport acceptance passes.
6. Lifted direction probe confirms left/right convention.
7. STOP, E-STOP, and TTL stop are verified on hardware.
8. Low-speed in-place active scan is verified.
9. Forward/backward remains disabled until a separate grounded safety gate is complete.

The intended future change boundary is adapters only. AutonomousSlamCoordinator, AutonomousSlamPolicy, and PreLiveAutonomousSlamGate stay stable.
