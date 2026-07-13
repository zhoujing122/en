# M3-D1.1 Runtime Sparse SLAM Wiring

M3-D1 completed the sparse three-ToF occupancy backend as an algorithmic backend. That did not by itself prove that the `robot_slamd` executable constructs or calls the new backend at runtime. M3-D1.1 adds an explicit, default-off Sparse Shadow runtime mode to prove runtime reachability without changing the legacy default path.

## Runtime Modes

`slam_runtime.mode` has two accepted values:

- `legacy`: default. The executable keeps the existing runtime path with `SensorManager`, `Localizer`, `TofFilter`, `ChunkedGrid`, existing scan/yaw correction observers, existing logging, and the null writer safety default.
- `sparse_shadow`: explicit shadow mode. The executable runs a separate in-memory sparse SLAM path and returns before constructing legacy mapping/correction objects.

Invalid mode strings are rejected. Sparse Shadow is never selected automatically from sensor source or other settings.

## BackendInput Pose Handoff

Sparse occupancy mapping requires an explicit pose. M3-D1.1 adds a map update input carrying `predicted_map_pose` and `has_predicted_map_pose`. The adapter maps those fields directly into `SlamBackendInputFrame.predicted_pose` and `SlamBackendInputFrame.has_predicted_pose`.

Missing pose is not filled with `(0,0,0)`. The old `integrate_sensor_snapshot(snapshot, now_s)` entry remains for compatibility, but it does not synthesize pose. When used with the sparse backend, missing pose is rejected by the backend.

## Sparse Shadow Runtime

Sparse Shadow constructs:

1. a canonical deterministic simulation `RobotSlamSensorPort` that produces native `RobotSlamSensorSnapshot` with `has_multi_tof=true`, wheel, and IMU fields;
2. `WheelImuDeadReckoning2D`;
3. `SparseMultiTofOccupancyBackendBinding`;
4. `SlamBackendMapPortAdapter`;
5. an explicit `RobotSlamMapUpdateInput` for each accepted odometry update.

The path is:

`real_main` -> runtime mode branch -> `SparseShadowRuntime` -> canonical sensor port -> `WheelImuDeadReckoning2D::update` -> `estimated_pose()` -> `RobotSlamMapUpdateInput.predicted_map_pose` -> `SlamBackendMapPortAdapter` -> `SlamBackendInputFrame.predicted_pose` -> sparse multi-ToF backend -> in-memory sparse occupancy grid.

## Temporary Frame Contract

M3-D1.1 does not implement `map_T_odom`. Sparse Shadow only supports a new empty sparse map under StartupZero:

- initial odom pose is `(0,0,0)`;
- initial sparse map pose is `(0,0,0)`;
- startup forward is defined as sparse map `+X`;
- startup left is sparse map `+Y`;
- `predicted_map_pose = odom_pose` only because map and odom are temporarily identity.

Initial yaw is a coordinate-frame definition. It is not measured by IMU. The current IMU provides yaw rate, not absolute yaw.

Existing maps, ConfiguredPose, relocalization, pose correction, and pose writeback are not supported in Sparse Shadow. M3-D2A0 is responsible for map/odom frames and timed pose buffering.

## Sensor Source Boundary

M3-D1.1 supports only deterministic canonical simulation source inside Sparse Shadow. Hardware and replay sources are fail-closed until a canonical no-hardware replay source is wired explicitly. Sparse Shadow does not open `/dev`, UART, I2C, SPI, or ioctl.

Sparse Shadow consumes native `multi_tof` snapshots. It does not read legacy `ranges_m`, does not fan out scalar ToF into beams, and does not infer NoReturn from confidence zero or maximum distance.

## Runtime Exclusions

Sparse Shadow does not construct or call:

- `ChunkedGrid`;
- `TofPoseCorrector`;
- `SpinScanLocalizer`;
- `SparseScanCollector`;
- `SparseScanYawMatcher`;
- yaw correction apply/writeback;
- recovery;
- motion execution;
- production map saving.

It writes only `sparse_shadow_runtime_report.txt` into the run directory. That report is not a map file.

## Safety

Default safety remains fail-closed:

- `motion_execution_mode=dry_run`;
- `hardware_write_enabled=false`;
- `allow_writer_dispatch=false`;
- `writer_backend=null`;
- production software motion interface disabled;
- forward/backward disabled;
- real hardware adapters disabled;
- pose and relocalization writeback disabled;
- yaw correction writeback disabled.

Sparse Shadow report fields state whether hardware, motion, map write, or pose writeback were attempted. The expected value for each is `false`.

## Not Implemented Here

M3-D1.1 does not implement local matching, keyframes, observation bundles, Timed Pose Buffer, `map_T_odom`, sparse map save/load, relocalization, Frontier, A*, path tracking, real sensors, or real motion.

Next stage: M3-D2A0 - Initialization, map/odom Frames and Timed Odom Pose Buffer.
