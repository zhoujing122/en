# M3-D2 Sparse ToF Local Matching and Keyframes

M3-D2 adds local sparse ToF matching, pose correction, and keyframe transactions on top of the M3-D1 sparse occupancy backend. It reduces Wheel/IMU dead-reckoning drift in bounded local conditions. It is not global relocalization, loop closure, pose graph SLAM, Frontier exploration, A*, or real robot enablement.

## Goal

M3-D1 consumes native three scalar ToF rays with a Wheel/IMU predicted pose and writes a sparse occupancy grid. M3-D2 separates odom and map frames, evaluates a bounded local SE(2) correction against an already committed reference map, and commits keyframe map updates transactionally.

The intended chain is:

`Wheel/IMU -> WheelImuDeadReckoning2D -> odom pose -> map_T_odom -> corrected map pose -> sparse ToF matcher -> keyframe batch map update`.

## Initial Pose Contract

The current IMU provides yaw rate, not absolute yaw. A stationary gyro reading of zero does not define world heading, and wheel odometry cannot measure absolute world direction. M3-D2 therefore treats initial yaw as a coordinate contract.

`StartupZero` creates a new map with odom pose `(0, 0, 0)`, map pose `(0, 0, 0)`, and startup forward as map `+X`. This yaw is explicitly defined by the new map frame, not measured by the IMU.

`ConfiguredPose` is used when an existing map has a known startup pose such as a fixed dock. The configured pose must be finite and the yaw is normalized.

An existing non-empty map without a configured initial pose is rejected with an initialization fault. M3-D2 does not perform 360 degree global yaw search or whole-map relocalization.

## Frames and SE(2)

`WheelImuDeadReckoning2D` outputs an odom-frame pose. The sparse occupancy map is in the map frame. M3-D2 maintains `map_from_odom`, also called `map_T_odom`.

The corrected map pose is:

`map_pose = map_T_odom compose odom_pose`.

The estimator is never written back. When a match proposes a corrected current map pose, M3-D2 derives a candidate `map_T_odom` from that matched pose and the current odom pose. This prevents repeatedly adding the same correction to the estimator.

The minimal SE(2) utility implements finite validation, yaw normalization, compose, inverse, between, and correction magnitude helpers. It reuses `RobotPose2D`; no second pose type is introduced.

## Matcher Input

The matcher uses a bounded multi-frame sparse ToF observation bundle. Each observation stores timestamp, odom pose, and up to three native scalar ToF ray observations. Invalid rays remain diagnostic only and do not score. The bundle has maximum observation and ray counts and enforces monotonic timestamps.

The bundle stores odom poses and sensor observations only. It does not store Plant pose, FakeWorld geometry, expected pose, or command-derived motion.

## Reference Map and Self-Matching Prevention

Matching reads a frozen snapshot of the already committed reference map. The active keyframe bundle is not written into that snapshot before scoring. The first keyframe on an empty map is a bootstrap transaction and does not call the matcher.

The required order is:

1. collect active observations;
2. freeze reference map snapshot;
3. score the active bundle against the frozen reference;
4. prepare pose correction;
5. prepare keyframe map update;
6. commit map;
7. commit `map_T_odom` correction;
8. clear the active bundle only after success.

This prevents the current bundle from matching against its own newly created cells.

## Cell Classes

The matcher reuses M3-D1 integer evidence and thresholds:

- `Unknown`: cell absent from the sparse map;
- `Free`: evidence at or below the free threshold;
- `Occupied`: evidence at or above the occupied threshold;
- `Uncertain`: existing cell between thresholds.

No separate matcher-only occupancy threshold is introduced.

## Scoring Model

Scoring uses integer raw scores and a deterministic normalized score. It evaluates:

- Hit free path consistency before the endpoint;
- Hit occupied endpoint agreement;
- NoReturn free-space path agreement;
- free/occupied contradictions;
- unknown and uncertain cell counts;
- evaluated ray count and known-cell ratio.

Invalid rays do not generate path cells, endpoints, or positive score. NoReturn rays never create occupied endpoint rewards. Unknown space is gated by known-cell ratio so an unknown map cannot produce a false high-confidence match.

The score output includes raw score, normalized score, known/unknown cell counts, known ratio, evaluated ray count, endpoint agreements, and free-path contradictions.

## Candidate Search

M3-D2 uses bounded deterministic coarse-to-fine local search. The coarse stage evaluates a finite dx/dy/dyaw window and keeps a deterministic top-K. The fine stage searches around those top candidates with smaller steps.

All configuration values are finite-checked. Translation and yaw steps must be positive. The candidate product is checked before evaluation and a maximum candidate count rejects oversized searches. There is no random sampling, particle filter, nonlinear optimizer, or 360 degree search.

Tie-break order is stable: normalized score, raw score, known cells, fewer contradictions, smaller correction magnitude, smaller yaw correction, smaller translation magnitude, dx, dy, dyaw, then generation index.

## Acceptance and Degeneracy

The matcher does not accept a candidate just because it is the highest-scoring sample. It gates on best score, known-cell ratio, valid ray count, hit ray count, score margin, and runner-up ambiguity. Runner-up candidates near the best correction are excluded from ambiguity margin so adjacent fine-grid samples do not create false rejection.

Observability probes compare score contrast along x, y, and yaw. `FullSE2` is accepted only when x, y, and yaw are observable. `YawOnly` is allowed when yaw is observable but translation is degenerate. `Rejected` is used for insufficient score, low overlap, ambiguity, or unobservable yaw.

This is important in corridors and symmetric rooms. A corridor may permit yaw-only correction while rejecting arbitrary longitudinal translation. Symmetric geometry with close far-away runner-up scores is rejected.

## Keyframes

A keyframe is a bounded time-contiguous group of sparse three-ToF observations and odom poses. It is not a pose-graph node and is not globally optimized.

Triggers include minimum snapshot count, translation, rotation, duration, maximum duration, and motion-settled events. The manager enforces timestamp monotonicity, bundle size limits, valid-ray minimums, and hit-ray minimums. Invalid rays do not increase valid counts.

Metadata records keyframe id, times, first/last odom poses, corrected last map pose, correction mode, correction deltas, scores, known ratio, counts, bootstrap/matched flags, and map commit status. Metadata history is bounded.

## Bootstrap

When the reference map is empty, the first ready keyframe is committed as bootstrap if initialization is valid and the bundle has enough information. Bootstrap uses the current `map_T_odom`; under `StartupZero` this is identity. It does not run the matcher and does not claim a pose correction.

Bootstrap still uses the same transactional sparse map update path. Capacity failure rolls back the whole bootstrap keyframe.

## Transactionality

Keyframe map updates are prepared as a batch. Every observation is transformed with the same candidate `map_T_odom`, all ray cell updates are generated, duplicates are resolved, occupied wins over free, capacity and traversal limits are checked, and only then is the sparse grid committed.

If map prepare fails, correction is not committed. If correction prepare fails, map is not committed. If matcher rejects, map and correction remain unchanged. Odom-only fallback after rejection is disabled by default.

## Local SLAM Pipeline

`SparseTofLocalSlamPipeline` receives odom pose, native multi-ToF snapshots, and phase flags. It does not command the robot, duplicate the Coordinator state machine, access Plant/FakeWorld, write real maps, or perform pose writeback.

During WaitingMotionSettle or active motion, Wheel/IMU odometry may continue outside this pipeline, but this pipeline does not add observations to the active keyframe, run matcher, or update the occupancy map. After settle, it accepts a phase-allowed multi-ToF snapshot, uses the latest odom pose, adds it to the active keyframe, and bootstraps or matches when ready.

## Deterministic Drift Injection

Closed-loop tests can inject deterministic wheel or IMU bias in the simulated measurements. The matcher sees only odom pose, sparse ToF observations, and the reference map. It does not receive the drift truth or ground-truth pose.

Ground truth is allowed only in test assertions and final evaluation metrics such as corrected-pose error or map-smearing reduction. It is not used as matcher input, pose input, or map update input.

## Map Smearing Reduction

M3-D2 compares odom-only mapping against corrected local SLAM in the same deterministic drift scenario. The accepted metric is based on resulting map consistency, occupied-cell spread, wall thickness, endpoint dispersion, or final pose error evaluated after the run. FakeWorld geometry and ground truth are never used to improve matcher scoring or map updates.

## Performance Boundary

The implementation is bounded for RV1126B constraints: at most three rays per frame, bounded active bundle, bounded candidate count, configurable path stride, bounded keyframe history, no random numbers, no sleeps, no thread pool, no ROS/PCL/Ceres/GTSAM/g2o/Eigen/OpenCV/Boost/protobuf dependency, and no per-frame file writes. Candidate scoring uses the frozen map snapshot and current candidate cells only; it does not copy the full map per candidate.

The code checks non-finite poses/configuration, zero steps, negative windows, candidate budget overflow, empty bundles, zero known-cell divisions, yaw normalization, and large correction bounds.

## Ground Truth Isolation

Matcher, correction state, keyframe manager, and local SLAM pipeline do not include `SimRobotPlant` or `FakeWorld2D`. They do not accept ground-truth pose parameters, command speed, command duration, target yaw, target distance, or expected pose sequences.

The valid algorithm chain is:

`SimRobotPlant -> SimWheelEncoder / SimImu -> WheelImuDeadReckoning2D -> odom_pose -> SparseTofPoseCorrectionState -> predicted map pose -> SparseTofLocalScanMatcher -> corrected map pose -> keyframe batch map update`.

Plant and FakeWorld are restricted to simulated sensor generation, test assertions, and final evaluation.

## Safety Boundary

Production safety defaults remain fail-closed: dry-run motion, null writer backend, no writer dispatch, no production interface, no translation commands, no forward/backward enablement, no real hardware adapters or stubs, no pose writeback, and no relocalization writeback.

M3-D2 does not access `/dev`, UART, I2C, SPI, ioctl, real ToF, real IMU, real BL4820, motor writers, real map paths, or pose writeback.

## Not Implemented

M3-D2 does not implement global relocalization, 360 degree search, loop closure, pose graph optimization, global optimization, Frontier exploration, A*, path tracking, autonomous exploration, or real robot motion.

## Next Stage

M3-E should add Frontier Exploration, A* Planning, and Safe Path Execution after the D2 local matching and keyframe commit boundaries are independently staged, verified, and committed.
