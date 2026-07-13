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
