# Full Autonomous SLAM Fake Pipeline

M3-B4 adds a fake/replay full autonomous SLAM pipeline. It is not live and not production SLAM. It does not read hardware, does not read SDKs, does not access serial devices, and does not move the robot.

The pipeline uses RealSensorReplayPort or deterministic fake replay data, DeterministicSlamBackendBinding, SlamBackendMapPortAdapter, AutonomousSlamCoordinator, AutonomousSlamPolicy, and FullAutonomousSlamFakeMotionPort. The goal is to close the autonomous SLAM loop before hardware exists.

Real hardware integration should replace only the sensor, motion, and SLAM backend adapters while preserving the coordinator, policy, pre-live runner, and main fake pipeline shape.
