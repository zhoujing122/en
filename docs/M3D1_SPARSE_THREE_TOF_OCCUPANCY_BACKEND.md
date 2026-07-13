# M3-D1 Sparse Three-ToF Occupancy Backend

M3-D1 adds the first native mapping backend for the three scalar ToF sensors. It is still offline, deterministic, and fake/simulation only. It does not enable real hardware or real motion.

## Goal

M3-D0 closed the loop between Coordinator commands, simulated motion, and simulated sensors. M3-D1 adds a lightweight sparse occupancy backend that consumes `snapshot.multi_tof` directly instead of the legacy front scalar projection.

## Return Semantics

Each scalar ToF mapping observation is classified as one of four return kinds:

- `Unspecified`: adapter did not provide an explicit mapping return kind, so the mapping builder derives one from the existing protocol and quality contract.
- `Invalid`: corrupted payload, invalid distance, low confidence, confidence zero, bad timestamp, sync failure, dropout, stale data, or otherwise unusable data.
- `Hit`: valid measurement with a usable occupied endpoint.
- `NoReturn`: valid measurement process with no occupied endpoint inside the configured free-space range.

`confidence` and return kind are separate. `Hit + confidence=100`, `NoReturn + confidence=100`, and `Invalid + confidence=0` are valid combinations. `confidence=0` is not a valid no-return signal.

## 9-byte Protocol

The real 9-byte payload remains unchanged: 48-bit echo tag, 16-bit distance in millimeters, and 8-bit confidence. The protocol does not contain a no-return flag. The real adapter does not infer `NoReturn` from distance 12000 mm, confidence zero, echo tag values, timeout guesses, or maximum range. Real 9-byte data defaults to `Hit` only when the existing protocol, confidence, range, timestamp, and sync gates pass.

Simulation may explicitly mark no-return samples through metadata. This keeps M3-D0 default no-hit behavior compatible: max-range plus confidence zero remains `Invalid` and diagnostic-only unless explicit no-return mode is enabled.

## Sparse Grid

The occupancy grid is sparse and deterministic. Unknown cells are absent from the container. World coordinates are converted with `floor(world / resolution)`, which is required for negative coordinates. Cell evidence is stored as bounded integer evidence, with configurable free and occupied deltas, thresholds, and saturation limits.

The grid has a maximum active-cell capacity. If a snapshot update would exceed capacity, the entire snapshot is rejected and the map remains unchanged.

## Ray Updates

Each scalar ToF contributes at most one center ray. The 1.6 degree FOV remains metadata only; M3-D1 does not generate multiple beams, angle fans, or point clouds.

- `Invalid`: no free update, no occupied endpoint, no quality success count.
- `Hit`: free cells are updated before the endpoint; the endpoint is updated occupied; the origin cell is skipped by default.
- `NoReturn`: cells along the free-space range are updated free; no occupied endpoint is written.

Ray traversal is deterministic and bounded. Cells are de-duplicated per ray and per snapshot. When rays overlap in one snapshot, occupied wins over free so update order cannot change the result.

## Transactionality

Snapshot update is transactional:

1. Validate backend readiness, timestamp, multi-ToF presence, and estimated pose.
2. Convert front/left/right scalar frames to sparse ray observations.
3. Drop invalid rays.
4. Build candidate cell updates for Hit and NoReturn rays.
5. Resolve duplicates and occupied-over-free conflicts.
6. Check traversal and map capacity limits.
7. Commit evidence updates only after all checks pass.

Rejected updates do not partially change the map and do not increment accepted update counters.

## Estimated Pose

The backend requires `SlamBackendInputFrame.has_predicted_pose=true`. That pose is supplied by `WheelImuDeadReckoning2D`, which only accepts `WheelOdomFrame`, `ImuFrame`, timestamps, and chassis/configuration limits. It does not accept motion commands, command duration, command speed, target yaw, target distance, `SimRobotPlant`, `FakeWorld2D`, M3-D0 reports, or ground-truth pose. The backend itself also does not include or read `SimRobotPlant`, `FakeWorld2D`, M3-D0 reports, or ground-truth pose.

The current backend-facing wheel contract exposes linear and angular velocity, not cumulative unwrapped left/right wheel position. Therefore M3-D1 integrates wheel linear velocity for translation and uses IMU yaw rate for yaw when IMU is fresh. Wheel angular velocity is used as a deterministic consistency gate and as an explicit fallback only when configured. The update is transactional: invalid timestamps, stale wheel data, excessive dt, excessive speed, yaw disagreement, or non-finite candidate pose leave the previous pose and previous samples unchanged.

M3-D1 does not implement scan matching or pose correction. The dead-reckoned pose can drift and is not localization confidence. M3-D2 is expected to correct this drift with sparse ToF scan matching and keyframes.

## Native Backend

`SparseMultiTofOccupancyBackendBinding` implements `SlamBackendBinding` and consumes `snapshot.multi_tof` natively. It does not require `snapshot.has_tof`, does not read `snapshot.tof.ranges_m`, and does not use the legacy scalar projection. Front, left, and right sensors are converted independently into one ray each.

AllowMissingOne degraded snapshots can still update the map when at least one valid Hit or NoReturn ray remains. A snapshot with no usable rays is rejected.

## Map Quality

Map quality is a readiness heuristic, not localization confidence. It is based on accepted snapshots, valid rays, observed cells, angular coverage bins, and capacity ratio. It is not hard-coded, does not use ground-truth error, and does not compare against `FakeWorld2D`.

## D0 Closed-loop Occupancy

The D1 closed-loop test reuses SimClock, FakeWorld2D, SimRobotPlant, SimMotionPort, SimSensorPort, AutonomousSlamCoordinator, `WheelImuDeadReckoning2D`, and the native sparse backend. Coordinator active scan commands cause the simulated plant to rotate. SimWheelEncoder and SimImu generate measurements from the plant, the estimator integrates those measurements every physics tick, and only `estimator.estimated_pose()` is copied into `SlamBackendInputFrame.predicted_pose`. The predicted pose is not computed from command speed, command duration, target yaw, target distance, prewritten pose sequences, Plant pose, or FakeWorld.

WaitingMotionSettle does not publish SLAM snapshots to the backend and does not update occupancy. Wheel/IMU odometry is still allowed to update internally during that phase so the next settled SLAM snapshot uses the latest estimated pose. After settle, publishing resumes and native three-ToF rays update the map. Replay is not used.

## Safety

Production defaults remain fail-closed: dry-run motion, null writer backend, no writer dispatch, no production interface, no translation commands, no forward/backward enablement, no real hardware adapters, and no pose or relocalization writeback.

M3-D1 does not access `/dev`, UART, I2C, SPI, ioctl, real ToF, real IMU, real BL4820, real motor writers, real map write paths, or real pose writeback.

## Performance Boundary

The implementation is intended for RV1126B constraints: no ROS, Gazebo, Webots, Box2D, Bullet, Boost, protobuf, GUI, thread pool, network service, graph optimizer, or large JSON dependency. Each frame processes at most three rays. Map storage is sparse and bounded, traversal is bounded, and normal updates only hold candidate cells for the current snapshot.

## Not Implemented

M3-D1 does not implement scan matching, pose correction, keyframes, pose graph, loop closure, Frontier, A*, autonomous exploration, real occupancy product output, or real robot operation.

## Next Stage

M3-D2 should add Sparse ToF Scan Matching, Pose Correction, and Keyframes on top of this occupancy backend.
