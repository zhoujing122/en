# M3-D1B0 Wheel/IMU Dead-Reckoning Pose Source

M3-D1B0 provides the `predicted_pose` source required by the native sparse three-ToF occupancy backend.

The estimator is intentionally small and deterministic. It lives outside the simulation tree so future real adapters can reuse it:

`include/robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp`

## Inputs

The estimator accepts only existing project data types:

- `WheelOdomFrame`
- `ImuFrame`
- `RobotPose2D` for reset
- `WheelImuDeadReckoningConfig`

It does not accept motion commands, command duration, command speed, target yaw, target distance, `SimRobotPlant`, `FakeWorld2D`, or ground-truth trace data.

## Current Wheel Contract

The published backend-facing wheel frame contains velocity, not cumulative wheel position:

- `timestamp_s`
- `linear_mps`
- `angular_rad_s`
- `valid`

Because no cumulative left/right wheel position is available in `WheelOdomFrame`, M3-D1B0 uses deterministic velocity integration:

```text
delta_s = wheel.linear_mps * dt
delta_yaw = imu.yaw_rate_rad_s * dt
```

Wheel angular velocity is used as a yaw consistency gate and as an explicit fallback only when configured.

## Integration

```text
yaw_mid = previous_yaw + 0.5 * delta_yaw
x_new = x_old + delta_s * cos(yaw_mid)
y_new = y_old + delta_s * sin(yaw_mid)
yaw_new = normalize(previous_yaw + delta_yaw)
```

The first valid sample only establishes a timestamp baseline. The second valid sample is the first one that can move the pose.

## Transactionality

Each update validates configuration, wheel freshness, IMU freshness or configured fallback, finite values, timestamp monotonicity, maximum `dt`, wheel velocity bounds, IMU yaw-rate bounds, and wheel/IMU yaw-rate consistency. If any check fails, the pose and previous samples are not changed.

## Ground Truth Isolation

The estimator does not include simulation headers. It cannot read plant pose or world geometry. Simulation may generate Wheel/IMU measurements from `SimRobotPlant`, but `predicted_pose` must be produced only by integrating those measurements.

M3-D1B keeps phase-aware SLAM publication separate from odometry sampling: Wheel/IMU odometry may update during `WaitingMotionSettle`, while the occupancy backend must not receive new mapping snapshots until the motion has settled.
