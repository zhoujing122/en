# M3-D0 Closed-Loop 2D Simulation

M3-D0 replaces prewritten post-motion replay data with a deterministic 2D robot simulation. The Coordinator still sees only normal motion feedback and `RobotSlamSensorSnapshot`; ground truth stays inside the simulation plant, sensors, tests, and report.

## Scope

- M3-C4 proved scalar three-ToF replay can enter the fake autonomous pipeline.
- M3-D0 proves Coordinator motion commands can move a simulated robot over multiple fixed ticks and that the changed pose changes wheel, IMU, and three scalar ToF observations.
- This is not real robot control, native three-ToF SLAM, occupancy mapping, scan matching, Frontier, A*, or autonomous exploration.

## SimClock

`SimClock` is caller-driven and deterministic. It accepts only finite positive `dt`, never sleeps, never reads wall clock, and uses seconds as the only unit.

## FakeWorld2D And Raycast

`FakeWorld2D` stores finite 2D line segments. It can add rectangular rooms and rectangular obstacles and performs a linear nearest-hit raycast over the small scene. Parallel lines, intersections behind the ray, out-of-range hits, endpoints, and no-hit cases are handled deterministically.

No-hit ToF semantics are unified: the diagnostic distance is max range, confidence is `0`, and the reading is not usable for mapping. It is not treated as a 12 m occupied wall.

## SimRobotPlant

The plant uses the existing `RobotPose2D` and a differential-drive state. It applies velocity and acceleration limits, midpoint-yaw integration, yaw normalization, continuous wheel position, and optional circle-footprint collision rejection. Collision rejects translation deterministically and does not implement rigid-body bounce.

## SimMotionPort

`SimMotionPort` implements `RobotSlamMotionPort`. It validates existing `AlgorithmMotionCommand` values through the Algorithm-to-Software adapter, then drives only the simulated plant.

Motion is non-instant:

- accepted commands set plant targets;
- the plant reaches targets over multiple fixed ticks;
- `command_active` remains true during execution;
- `command_settled` requires the command to finish and actual plant speed to remain below thresholds;
- `Stop` drives targets to zero;
- `EmergencyStop` latches simulated safety stop semantics.

Forward and Backward are controlled by `simulation.allow_translation`. This does not change production defaults: production translation, Forward/Backward, writer dispatch, and hardware writes remain disabled.

## Sensors

`SimWheelEncoder` generates parsed wheel velocity and pair timing from plant wheel state. It preserves request/response midpoint and pair timestamp semantics without encoding BL4820 UART bytes.

`SimImu` generates yaw rate from plant angular velocity and acceleration from linear velocity change. Bias is configurable; random noise is off by default.

`SimThreeScalarTof` casts exactly one center ray for each sensor:

- front: robot yaw;
- left: robot yaw + pi/2;
- right: robot yaw - pi/2.

It does not generate beams, an angle fan, a point cloud, or a fake laser scan. The 1.6 degree FOV is metadata only. Echo tags are deterministic identifiers and are not timestamps.

`SimSensorPort` implements `RobotSlamSensorPort`, generates wheel, IMU, and three scalar ToF from the current plant state, then calls the existing `MultiTofSnapshotBuilder`. Failures return a new empty snapshot and do not reuse a previous success. A call at the same simulation time is rejected instead of faking a fresh measurement.

## Closed Loop

`M3D0Runner` is a simulation scheduler, not a second Coordinator. Each fixed tick updates motion feedback, publishes a SLAM snapshot only in phase-allowed states, calls the existing `AutonomousSlamCoordinator`, steps the plant, and advances `SimClock`.

During `SendingMotionCommand` and `WaitingMotionSettle`, the plant and physical sensor state can continue evolving, but no new `RobotSlamSensorSnapshot` is published to the Coordinator. After simulated motion settles, a new snapshot is published from the changed pose.

## Ground Truth Boundary

Only FakeWorld, SimRobotPlant, simulation sensors, tests, trace, and M3-D0 reports can read ground truth. Coordinator, MapPort, and deterministic backend receive only motion feedback and sensor snapshots. The backend does not use plant pose to estimate pose or map quality.

## Safety

M3-D0 keeps production fail-closed:

- `motion_execution_mode=dry_run`
- hardware writes disabled
- writer dispatch disabled
- writer backend null
- production motion interface disabled
- translation commands disabled
- Forward/Backward disabled
- real hardware adapters disabled
- pose writeback disabled

The simulation code does not open `/dev`, UART, I2C, SPI, ioctl, or real motor writers.

## Tests

The M3-D0 tests cover deterministic clock advancement, raycast geometry, plant kinematics, collision, non-instant motion, Stop, EmergencyStop, simulation-only translation, sensor timing, stale rejection, sync rejection, closed-loop active scan, repeatability, report text, and production safety defaults.

## Next Stage

M3-D1 is the lightweight sparse three-ToF occupancy backend. M3-D0 intentionally does not implement that backend.
