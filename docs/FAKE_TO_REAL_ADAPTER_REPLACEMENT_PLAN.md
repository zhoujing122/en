# Fake To Real Adapter Replacement Plan

Future live work should replace RealSensorReplayPort with RealTofImuWheelSensorPort, FullAutonomousSlamFakeMotionPort with RealSoftwareMotionPort, and DeterministicSlamBackendBinding with RealSlamBackendBinding.

Fake map save remains disabled until real map storage is implemented. Real relocalization and pose writeback remain out of scope. AutonomousSlamCoordinator, AutonomousSlamPolicy, PreLive runner, and E2E runner should not need broad rewrites for adapter replacement.

## M3-B5 Fake Map Replacement

The in-memory fake map artifact is the placeholder for future map storage. Real storage should replace FakeMapStorage while preserving the completed-pipeline contract and without changing AutonomousSlamCoordinator or AutonomousSlamPolicy.
