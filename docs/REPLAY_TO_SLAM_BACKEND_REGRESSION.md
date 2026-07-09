# Replay To SLAM Backend Regression

M3-B3 adds an offline regression path:

`RealSensorReplayPort -> RobotSlamSensorSnapshot -> SlamBackendInputFrame -> DeterministicSlamBackendBinding::update -> RobotSlamMapQuality`

The valid replay log must produce accepted backend updates and increase deterministic map quality counters. Invalid replay logs, including high request latency and large sensor sync errors, must not update the backend map state.

This runner uses inline deterministic replay data. It does not read replay files by default, does not access hardware, does not connect to a production SLAM backend, and does not send motion commands.

The runner is a preparation point for future real SLAM backend regression. Once real golden logs exist, they can be replayed through the same binding contract before any live robot use.
