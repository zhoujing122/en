# M3-D2A0 Unified Sparse SLAM Initialization, Frames and Timed Pose Alignment

M3-D2A0 moves the sparse SLAM path toward one final runtime core. The
transitional `SparseShadowRuntime` remains a safe wrapper, but algorithm state
is being factored into reusable sparse SLAM contracts that future replay,
simulation, and production modes must share.

## Initialization and Frames

`RobotPose2D` remains the common numeric storage for `x_m`, `y_m`, and
`yaw_rad`, but sparse SLAM code must not pass bare poses across frame
boundaries. M3-D2A0 introduces:

- `OdomPose2D`: `odom_T_base`.
- `MapPose2D`: `map_T_base`.
- `MapFromOdom2D`: `map_T_odom`.

`StartupZero` for a new sparse map defines the startup robot pose as the map
origin and the startup forward direction as map `+X`. The initial yaw value of
zero is a coordinate definition, not an IMU measurement. The current IMU field
is `yaw_rate_rad_s`; it does not provide absolute startup yaw.

`ConfiguredPose` is supported for new maps by initializing `map_T_odom` to the
explicit configured map pose while keeping odometry reset at identity. This does
not write the configured pose into `WheelImuDeadReckoning2D`.

Existing sparse map load and relocalization are intentionally fail-closed in
this stage. Loading an existing map with a configured pose returns
`ExistingMapLoadNotImplemented`; relocalization returns
`RelocalizationNotImplemented`; an existing map without an initial pose returns
`InitialPoseMissing`.

M3-D2A0 does not implement local matching, keyframes, sparse map save/load,
relocalization, Frontier, A*, real hardware, or real motion.

## Timed Pose Alignment

TimedOdomPoseBuffer stores a bounded, strictly monotonic sequence of odom_T_base samples. It rejects non-finite timestamps, non-finite poses, duplicate timestamps, and non-monotonic timestamps. Lookup never extrapolates: queries before the oldest sample, after the newest sample, or across a gap larger than the configured interpolation limit are rejected. Interpolation is linear for x_m and y_m; yaw uses the shortest angular delta across the +/-pi boundary.

MultiTofMeasurementPoseResolver resolves each scalar ToF route independently: front uses front.effective_timestamp_s, left uses left.effective_timestamp_s, and right uses right.effective_timestamp_s. The resolver returns the robot base pose in the map frame at each measurement time. Sensor extrinsics remain in the sparse ToF observation builder/backend path and are not applied by the resolver.

RobotSlamMapUpdateInput and SlamBackendInputFrame now carry an explicit MultiTofMeasurementPoseSet. The sparse backend can run in strict measurement-time mode, where missing or timestamp-mismatched per-route poses reject the update. The legacy D1 single-pose fallback remains available only as an explicit compatibility option; the unified sparse runtime must disable it.


## Unified Runtime Core

`SparseSlamRuntimeCore` is the single sparse SLAM algorithm owner for this
migration path. It owns initialization state, `WheelImuDeadReckoning2D`,
`MapOdomFrameState`, `TimedOdomPoseBuffer`, `MultiTofMeasurementPoseResolver`,
`SlamBackendMapPortAdapter`, and `SparseMultiTofOccupancyBackendBinding`.
Replay, simulation, and future production sparse runtime modes must reuse this
core instead of creating separate estimator, frame, backend, or pose handoff
logic.

`SparseShadowRuntime` is transitional. It constructs a deterministic canonical
sensor source, creates the core, calls `initialize`, steps snapshots through the
core, and writes a report. It does not own a second estimator, a second sparse
backend, matcher state, keyframes, map save/load, relocalization, hardware, or
motion.

## Startup Gate

Startup initialization is sensor based. Wheel and IMU samples must be fresh and
finite. The startup sample must be stationary according to configured linear
speed, wheel yaw-rate, and IMU yaw-rate thresholds. Gyro bias is computed from a
bounded startup sample window and checked against absolute-bias and spread
limits. The wheel baseline is established before mapping begins. No sparse map
update is allowed before initialization is complete.

## Runtime Step

After initialization, each legal Wheel/IMU sample updates odometry and appends an
`odom_T_base(t)` sample to the timed pose buffer. If a native three-ToF snapshot
is present, the core resolves front, left, and right measurement-time base poses
separately, composes them through `map_T_odom`, and sends the resulting
`MultiTofMeasurementPoseSet` through `RobotSlamMapUpdateInput` and
`SlamBackendInputFrame` to the sparse backend. The core runs the backend in
strict measurement-time mode, so the D1 single-pose fallback is disabled in the
unified runtime.

Odometry update and map update are separate transactions. A valid odometry update
may remain committed when a ToF pose lookup or backend update is rejected. A map
failure does not write back into `WheelImuDeadReckoning2D`, and this stage does
not apply any matcher correction to `map_T_odom`.

## Migration Target

`SparseSlamRuntimeCore` becomes the single production runtime core. Legacy SLAM
and Sparse Shadow remain only for regression and safe migration. The legacy
Localizer, ChunkedGrid, TofPoseCorrector, SparseScanYawMatcher, and yaw
correction writeback remain outside the new sparse core and are expected to be
removed only after the D2 matcher/keyframe work, D3 map lifecycle work, and M3-F
simulation acceptance gates pass.
