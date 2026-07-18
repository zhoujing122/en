# Stop-go three-ToF straight-wall mapping foundation (P2-P3)

This worktree is `$HOME/Desktop/小恩/worktrees/en_stop_go_wall_mapping_p2`, on
`feature/stop-go-straight-wall-mapping-p2`, based on
`17162e8ccde4f6163012499e6ed470cf506c5843` (`consolidation/single-production-system`).
The existing single-system worktree and all motor worktrees remain untouched.

## Scope and composition

The production composition root remains:

```text
main
  -> RobotSlamApplication
     -> SparseSlamRuntimeCore
```

Core initialization is not duplicated by the stop-go controller. Startup is
ordered as:

```text
RobotSlamApplication startup
  -> SparseSlamRuntimeCore::initialize
  -> stationary Wheel/IMU snapshots
  -> gyro bias and wheel baseline
  -> odom_T_base and map_T_odom initialization
  -> Core localization_ready() == true
  -> StopGoMappingController INITIAL_SETTLE
  -> INITIAL_SAMPLE
  -> first relative motion command
```

The controller begins in `WAITING_FOR_CORE_READY`. While Core is collecting
stationary samples it continues passing valid canonical Wheel/IMU snapshots to
the existing Core, submits zero motion commands, and does not treat the normal
waiting result as a controller fault. Invalid startup samples keep the
controller waiting; a terminal Core initialization fault is reported as a
controller fault. The controller never calls estimator reset, clears encoder
ticks, creates `MapOdomFrameState`, changes `map_T_odom`, or writes a per-step
encoder baseline into SLAM.

The new simulation path is:

```text
RelativeMotionStepPort
  -> StopGoMappingController
     -> wait for one motion command
     -> MappingSettleGate
     -> StableThreeTofSampler
     -> canonical RobotSlamSensorSnapshot
     -> RobotSlamApplication
     -> SparseSlamRuntimeCore
```

The controller does not create a map, a second odometry source, or a second
`map_T_odom`. During motion and settling it submits wheel/IMU-only snapshots;
the ToF fields are removed before that snapshot reaches the core. Only the
stable stop sample can increment the canonical map revision.

## Nuona 11-byte ToF protocol

The parser accepts exactly:

```text
0x20 ddddd 0x2c ccc 0x0a
```

where distance is five ASCII digits in 20..4500 mm and confidence is three
ASCII digits in 0..100. It validates length, header, delimiter, trailer,
digits, overflow-safe range, and confidence range. It never uses `atoi`, and
it does not invent a `NoReturn` value: malformed or out-of-range data is
`Invalid`; a valid distance with confidence zero is still a valid parsed
distance, but it is not mapping-usable under the configured mapping filter.
`NoReturn` remains a Simulation-only semantic.

The parser has no clock. A Reader creates `TimedTofFrame` using the request
window midpoint:

```text
sample_timestamp_us = request_start_us
                    + (response_received_us - request_start_us) / 2
```

This is monotonic time, not UTC and not an echo field. No Linux I2C address is
assumed; the possible `0xCA/0xCB` byte values are not used as a device address.
There is no real I2C adapter in this phase.

## Stable three-ToF sampling

At each stopped sampling point, front/left/right each receive five frames.
At least three valid protocol frames are required. Valid distances are sorted
and the actual middle frame is selected. Its own distance, confidence, and
request-window midpoint are retained; no average distance or synthetic time is
created. A failed direction is Invalid, never filled from an older sample.

Protocol limits and mapping limits are separate. The current Simulation
configuration is:

```text
samples_per_sensor: 5
min_valid_samples: 3
protocol: 20..4500 mm
mapping: 30..4500 mm
mapping_min_confidence: 70
```

The confidence threshold is a provisional value and has not been validated on
the real vehicle.

## Settling semantics

P1 motor `DONE` means target reached, STOP succeeded, and the motor-side wheel
RPM entered its three-frame quiet condition. P2 adds `MappingSettleGate`:

```text
MOTOR_SETTLED
  + both wheel RPM <= 1.0 RPM for 3 frames
  + IMU gyro_z <= 1.0 deg/s for 3 frames
  + valid wheel/IMU timestamps and pair skew <= 10 ms
  + final 200 ms hold
  = MAPPING_READY_SETTLED
```

Timestamp order, wheel/IMU validity, and pair skew failures reset or reject the
gate. This phase does not implement IMU hardware access, and it does not claim
that a mapping sample is physically repeatable on the real chassis.

## Controller state machine

```text
IDLE
 -> WAITING_FOR_CORE_READY
 -> INITIAL_SETTLE
 -> INITIAL_SAMPLE
 -> ISSUE_FORWARD_STEP
 -> WAIT_MOTION_DONE
 -> WAIT_MAPPING_SETTLE
 -> ACQUIRE_THREE_TOF
 -> COMMIT_MAPPING_SAMPLE
 -> CHECK_LIMIT
 -> ISSUE_FORWARD_STEP ...
 -> COMPLETED / FAULT / ESTOP
```

Each step creates a new nonzero command ID and sends one FORWARD command in
millimetres. It waits for that command's result, settles, samples all three
ToF sensors, submits one canonical snapshot to the existing application,
records the map revision, and only then permits the next command. The
Simulation plant supplies feedback; the controller never uses plant ground
truth or requested distance as odometry. `map_writes_while_moving` is checked
as a revision guard and is expected to be zero. The Core's cumulative wheel
feedback, `last_odom_pose`, pose buffer, and `map_T_odom` remain continuous
across commands; the command-port baseline is used only for that command's
progress/result. `map_T_odom` is exposed to the controller only as a read-only
Core value and may be changed only by Core matching or relocalization logic.

The deterministic straight-wall scene starts parallel to a continuous left
wall, with open right space and no corner. It runs the provisional example
settings of 20 mm, 20 steps, 400 mm total, 12 RPM, and a 3000 ms command
timeout. These are Simulation parameters, not production calibration.

## Relative motion ports

`RelativeMotionStepPort` is neutral and contains no UART or register symbols.
Its action values are fixed as follows:

```text
0 STOP
1 RIGHT
2 LEFT
3 FORWARD
4 BACK
5 ESTOP
```

`SimRelativeMotionStepPort` drives `SimRobotPlant`; `ReplayNoMotionStepPort`
rejects ordinary motion and accepts no-motion STOP/ESTOP semantics; and
`RealRelativeMotionStepPort` is explicitly fail-closed. This worktree does not
depend on an absolute motore path.

## Replay and logs

Simulation writes `stop_go_mapping.jsonl` and a compact
`stop_go_mapping.replay` record. Replay carries command ID/state, requested and
actual motion values, wheel/IMU feedback, pair skew, parsed ToF values,
selected timestamps, request/response timestamps, valid counts, and map
revision before/after. Replay inserts odometry-only pre-roll/intermediate
feedback frames so the existing timed pose buffer remains valid; it never
sends motion.

Run Simulation:

```sh
cmake -S . -B "$HOME/Desktop/小恩/builds/en_stop_go_wall_mapping_p2" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$HOME/Desktop/小恩/builds/en_stop_go_wall_mapping_p2" -j4
timeout 20s "$HOME/Desktop/小恩/builds/en_stop_go_wall_mapping_p2/robot_slamd" \
  --config config/stop_go_straight_wall_sim.yaml \
  --output-dir /tmp/robot_slamd_stop_go_formal
```

Use the generated `stop_go_mapping.replay` with a replay configuration whose
`runtime.sensor_source` is `replay`, `runtime.operation` is
`stop_go_wall_mapping`, and `runtime.replay_path` names that file. Replay is
read-only with respect to motion and hardware.

## Explicitly not included

- no real I2C;
- no real UART or motor write;
- no ToF during-motion map writes;
- no left-wall correction or wall-line fitting;
- no corner detection or RIGHT 90-degree behavior;
- no IMU hardware settle confirmation beyond the abstract Simulation/Replay input;
- no ToF, wall-following, frontier, A*, or robot_slamd integration beyond the
  existing single Application/Core composition;
- no production parameter calibration.

The next phase should add a left-wall point window, wall-line fitting,
heading/distance error, and small LEFT/RIGHT correction steps over a 1..2 m
straight wall. Real experiments must first establish motor direction, 10/20/30/
50 mm repeatability, angle repeatability, slip, and settle timing. The current
Simulation does not prove that 10 mm or 0.5 degrees is repeatable on the real
chassis.
