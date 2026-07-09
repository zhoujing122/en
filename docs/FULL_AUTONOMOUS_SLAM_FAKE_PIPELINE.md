# Full Autonomous SLAM Fake Pipeline

M3-B4 adds a fake/replay full autonomous SLAM pipeline. It is not live and not production SLAM. It does not read hardware, does not read SDKs, does not access serial devices, and does not move the robot.

The pipeline uses RealSensorReplayPort or deterministic fake replay data, DeterministicSlamBackendBinding, SlamBackendMapPortAdapter, AutonomousSlamCoordinator, AutonomousSlamPolicy, and FullAutonomousSlamFakeMotionPort. The goal is to close the autonomous SLAM loop before hardware exists.

Real hardware integration should replace only the sensor, motion, and SLAM backend adapters while preserving the coordinator, policy, pre-live runner, and main fake pipeline shape.

## M3-B5 Trace And Fake Map

M3-B5 adds phase-aware replay consumption, a structured per-step trace, and an in-memory fake map artifact built after successful completion. The fake map is metadata only; no real map file is written.

## M3-B6 Relocalization Loop

After a successful fake pipeline run, M3-B6 can load the fake map metadata and evaluate one replay scan through FakeRelocalizationBinding. The result is report-only and never writes pose.

M3-B7 product acceptance composes this fake pipeline with fake map load and strict fake relocalization, still without live hardware or production SLAM claims.
