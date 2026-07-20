# Stop-Go Left-Wall Follow Mapping P4

## Scope and composition

P4 is based on `807343b900f339a39e11577e143cf05dd39212fa` (`Gate stop-go mapping on Core readiness`) in worktree `en_stop_go_left_wall_follow_p4`, branch `feature/stop-go-left-wall-follow-p4`.

The production call chain remains:

`Simulation / Replay Adapter → canonical RobotSlamSensorSnapshot → RobotSlamApplication → SparseSlamRuntimeCore → StopGoMappingController → RelativeMotionStepPort`.

There is still one Application and one SLAM Core.  The original Core initialization is preserved: startup stationary Wheel/IMU samples, gyro bias, wheel baseline, `odom_T_base`, `map_T_odom`, and `localization_ready()`.  Stop-Go waits in `WAITING_FOR_CORE_READY`, sends no motion commands during that wait, and feeds odometry-only stationary samples.

## Frames and measurement provenance

The base frame uses `+x` forward, `+y` left, and positive yaw to the left.  The configured planar extrinsic is used once as `base_T_left_tof`.  A wall point is computed only after the motor is Done, `MappingSettleGate` reports settled, the stable left sample is valid, and the Core accepts the map update:

`p_map = map_T_base_at_left_measurement ⊕ base_T_left_tof ⊕ (distance_m, 0)`.

The Core exposes the narrow read-only `LastAcceptedMapObservationView`, including the accepted snapshot identity, all measurement-time poses, map revisions, backend acceptance, and `frame_transform_epoch`.  The Controller reuses its left measurement pose; it does not project an old sample from the latest pose.

`frame_transform_epoch` starts at the successful startup frame, increments only when Core commits a changed `map_T_odom` during local matching/relocalization, and does not change for ordinary map-cell revisions.  A changed epoch clears the bounded wall window and invalidates the model.  `wall_model_reset_due_to_frame_change_count` records this guard.

## Wall fitting and control

`LeftWallLineEstimator` uses bounded 2-D PCA/TLS: centroid, 2×2 covariance, principal direction, baseline, orthogonal residuals, median/MAD outlier rejection, and a second fit.  NaN/Inf values, insufficient inliers, insufficient baseline, excessive RMS, and a non-left signed distance are rejected.  The tangent is oriented so its dot product with the current robot forward direction is non-negative.  The left normal is `(-sin(heading), cos(heading))` and the signed base-to-wall distance must be positive.

The wall target is either configured or captured once on the first valid model.  Control uses fitted wall heading and signed base-to-wall distance.  Distance error produces a bounded heading bias; heading error and the biased target produce a bounded turn error.  Positive turn error sends `LEFT` (action 2); negative turn error sends `RIGHT` (action 1).  Thus a heading toward the left wall turns RIGHT, a heading away turns LEFT, too far produces LEFT bias, and too close produces RIGHT bias.  At most one correction is scheduled between forward steps.

Wall acquisition permits bounded small forward steps until the configured fit point and baseline gates are met.  It never corrects from an immature model.  The correction lifecycle is:

`ISSUE_TURN_CORRECTION → WAIT_TURN_DONE → WAIT_TURN_SETTLE → VERIFY_AFTER_TURN → ISSUE_FORWARD_STEP`.

The command ID and action are checked on every motion result.  Verification takes a fresh stable three-ToF sample, checks the front safety and left wall, checks Core measurement-pose processing, and invokes Core with `mapping_write_enabled=false`.  Consequently `map_writes_during_turn_or_verify` is expected to remain zero.

## Safety and terminal behavior

Invalid left samples are retried while stopped and then terminate in `WALL_LOST`; old wall distances, right ToF, or “which side is farther” are never substituted.  A front Hit below the stop threshold terminates in `FRONT_BLOCKED`; invalid/unknown front readings are retried with a bounded limit.  P4 does not perform a corner turn or a RIGHT 90° maneuver.

## Simulation, Replay, and configuration

`config/stop_go_left_wall_follow_sim.yaml` selects `mode: left_wall_follow`, a 150 mm configured base-to-wall target, a ten-point window, five-point/50 mm fit gates, 30 mm RMS, 20 mm minimum outlier threshold, 3× MAD, 1° heading deadband, 10 mm distance deadband, 20°/m distance gain, ±3° bias, 1–3° correction, 50–500 mm safety bounds, and a 200 mm front stop threshold.  All values are parsed, finite/range checked, and bounded.  The existing P2 configuration remains the `straight` mode.

The Formal runner covers nominal, heading toward/away, too far/too close, left-wall loss, and front-blocked scenarios.  Replay uses the existing P2 log/codec and no-motion replay path; it carries wall points, window-fit values, epoch, and control decisions and verifies deterministic map reconstruction.

Real sensor and motion sources remain fail-closed.  No real I2C, UART, or motor write is enabled by P4.  The formal tests do not prove real-robot 1° heading, 20 mm distance, or any threshold calibration; these are provisional simulation parameters only.

## Next stage

The next stage is wall-corner recognition, a fixed RIGHT 90° action, acquisition of the new left wall, and a single-corner validation.  Full rectangular-room traversal, continuous full-SE(2) matching, pose graph, and loop closure remain out of scope.
