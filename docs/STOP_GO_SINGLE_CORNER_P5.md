# Stop-Go Single-Corner P5

Baseline: `20f88f15dc754461bb4bc218cf0bcb7685b2284c` (`Add stop-go left-wall fitting and correction loop`).

This checkpoint is implemented in worktree
`/home/lab/Desktop/小恩/worktrees/en_stop_go_single_corner_p5` on branch
`feature/stop-go-single-corner-p5`.

## Ownership and initialization

The only runtime chain remains:

`Simulation / Replay Adapter -> RobotSlamSensorSnapshot -> RobotSlamApplication -> SparseSlamRuntimeCore -> StopGoMappingController -> RelativeMotionStepPort`.

P5 retains one `RobotSlamApplication`, one `SparseSlamRuntimeCore`, one
`WheelImuDeadReckoning2D`, one `TimedOdomPoseBuffer`, one `map_T_odom` owner and
one sparse-map owner. Core initialization still consumes stationary Wheel/IMU
samples, estimates gyro bias, establishes the wheel baseline, initializes odom
and `map_T_odom`, and only then reports `localization_ready`. The controller
remains `WAITING_FOR_CORE_READY` with zero submitted motion commands until that
gate passes. Lost/recovery/epoch changes stop and cancel the corner flow.

## Corner flow

The controller follows the first left wall through bounded PCA/TLS wall fitting,
then requires a valid wall model, stable front/left Hit samples, confidence and
the configured trigger distance before entering `CORNER_CANDIDATE` and
`CORNER_CONFIRMING`. Confirmation is stationary, requires multiple independent
stable samples, disables mapping writes, and rejects a one-sample false front
return.

Before the fixed turn, the right StableThreeTof sample is a clearance gate only;
it never selects direction. The safety inequality is:

```text
base_to_front_wall = front_tof_forward_offset + front_distance
base_to_front_wall >= turning_sweep_radius + clearance_margin
trigger_distance >= max(0, turning_sweep_radius + clearance_margin - front_tof_forward_offset)
                   + forward_step + expected_forward_overrun
```

P5 sends exactly one `RIGHT` (`action=1`) command with `90` degrees. It waits
for the matching command ID, motor Done and `MappingSettleGate`. Turn success is
judged only from Core's continuous `odom_T_base` yaw. The measured delta is
compared with `-pi/2`; a residual is corrected with bounded RIGHT/LEFT commands,
each with its own command ID, settle and re-verification. Command amount is not
used as actual odometry.

After angle verification, P5 performs a mapping-disabled stationary sensor
verification. It checks Core readiness, frame epoch, front safety and the new
left-wall observation before entering `NEW_LEFT_WALL_REACQUISITION`. The old
point window/model is cleared, `wall_segment_id` changes from 1 to 2 and
`corner_transition_id` increments. The frame epoch does not change merely due
to this ordinary right turn. A new accepted measurement-time left pose is
projected through `base_T_left_tof`; the new bounded wall window is fitted and
normal left-wall follow resumes for the configured number of steps. P5 then
terminates with `single_corner_complete`, never attempting a second corner.

All accepted left points use the Core `LastAcceptedMapObservationView` (or its
equivalent) and the exact measurement-time base pose used by the map update.
Controller code never writes `map_T_odom`, resets odometry, clears Core pose
history, writes the sparse map directly, or consumes simulation ground truth.

## Simulation, Replay and parameters

`config/stop_go_single_corner_sim.yaml` is the formal Simulation configuration.
The deterministic L-shaped fixture has one horizontal first wall and one
perpendicular wall that becomes the new left wall after RIGHT 90°. Formal cases
cover nominal, under-rotation with bounded RIGHT residual correction,
over-rotation with bounded LEFT correction, blocked right clearance, a false
front measurement, missing new wall and failed turn verification.

The P5 corner values are Simulation provisional: 80 mm sweep radius, 20 mm
margin, 100 mm front-ToF offset, 10 mm expected overrun, 350 mm trigger,
200 mm right clearance, 90 degree main turn, 3 degree acceptance, 8 degree
maximum residual, two residual attempts, 80--500 mm new-wall range, eight new
wall acquisition steps and five post-corner follow steps. The Simulation uses a
6 s motion timeout because the provisional 12 RPM model needs more than 3 s for
a 90 degree rotation. Production Real mode remains fail-closed and does not
inherit these values automatically.

Replay extends the existing P4 JSONL/replay codec. It carries candidate and
confirmation events, clearance, command IDs, Core odom yaw, measured turn delta,
residual corrections, sensor verification, segment transition, new-wall model,
post-corner follow count, map revisions and termination. Replay sends no motion
commands. Two nominal Simulation runs and two Replay runs reproduce command,
point/model hashes, estimated pose, map revision, checksum and termination.

## Scope and limitations

P5 proves the single-corner loop in Simulation and Replay only. It does not prove
real 90-degree accuracy, turning sweep radius, motor overrun, right-side safety
clearance, ToF calibration, or new-wall acquisition distance on hardware. It
does not implement LEFT 90 degrees, automatic four-corner traversal, loop
closure, pose graph, a second SLAM/map/odometry path or a final pose snap.

P6 is planned for multiple wall segments, four corners, return-near-start and
full rectangular-map acceptance.
