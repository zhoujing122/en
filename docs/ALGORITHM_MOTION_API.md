# Algorithm Motion API

This document defines the algorithm-layer motion command facade for Xiao En `robot_slamd`.
It is a high-level command contract for software-side chassis control. It is not a BL4820
register protocol, not a UART speed command format, and not a PWM/FR interface.

## Scope

The algorithm layer expresses motion intent as `AlgorithmMotionCommand`, then converts it to
`SoftwareMotionCommand` through `AlgorithmMotionCommandAdapter`. A shadow or future production
`SoftwareMotionCommandTransport` is responsible for accepting or rejecting the resulting command.

The algorithm layer does not:

- Write motor registers.
- Write BL4820 speed commands.
- Send UART motion commands.
- Publish real socket/ROS/LCM motion commands.
- Bypass `MotionSafetyExecutor` or `MotionWriteController` in production paths.

## High-Level Commands

The API includes:

- `stop`
- `emergency_stop`
- `direction_probe_left`
- `direction_probe_right`
- `active_scan_turn_left`
- `active_scan_turn_right`
- `recovery_forward`
- `recovery_backward`
- `recovery_turn_left`
- `recovery_turn_right`
- manual test turn/translation builders for test and future guarded use

`FORWARD` and `BACKWARD` are represented in the API but remain disabled by default. Current active
scan behavior is rotation/stop only.

## Command Fields

Each non-stop command carries:

- `kind`
- `profile`
- `speed_normalized`
- `duration_s`
- `timestamp_s`
- `ttl_s`
- `reason`
- `sequence`

`speed_normalized` remains in `[0.0, 1.0]`. Profile-specific defaults and limits are intentionally
small: direction probe and manual test default to `0.03`, active scan and recovery default to `0.05`.

## TTL And Stop Semantics

Software-side execution must enforce `ttl_s`: if no fresh command arrives before TTL expiry, the
chassis control layer must stop. `STOP` must be idempotent and safe to repeat. `EMERGENCY_STOP` has
the highest priority.

## Transport Semantics

A software transport must return accepted/rejected status. A rejected or failed send keeps the
algorithm-side result failed and must not be treated as movement permission.

## Direction Probe

`DirectionProbeLeft` and `DirectionProbeRight` exist to verify the direction convention while the
robot is lifted or wheels are free. The direction convention must be verified before any grounded
motion. See `docs/M2B1_LIFTED_LOW_SPEED_LIVE_TEST_CHECKLIST.md`.

## Software-Side Implementation Guidance

Software-side chassis code should implement `SoftwareMotionCommandTransport` or an equivalent adapter
that consumes `SoftwareMotionCommand`:

- direction
- speed_normalized
- timestamp_s
- ttl_s
- source
- reason
- sequence

Software-side code should not require algorithm modules to provide left/right wheel RPM. The older
`SoftwareDirectionSpeedMotionCommandWriter` remains for compatibility with `MotionWriteController`,
but new algorithm modules should prefer `AlgorithmMotionFacade` for high-level intent.

## Current Stage

M2-B2 only defines and tests the high-level API, builder, adapter, and facade. Production remains
fail-closed. No real transport is enabled and Xiao En must not move in this stage.

## M2-B3 Software Transport Handoff

M2-B3 adds `SOFTWARE_TRANSPORT_IMPLEMENTATION_SPEC.md`, `SOFTWARE_TRANSPORT_ACCEPTANCE_TESTS.md`, and `SOFTWARE_TRANSPORT_GOLDEN_COMMANDS.md`. Software-side live transport must implement the direction + speed + duration + ttl contract; the algorithm layer still does not output BL4820 register writes, UART speed commands, PWM/FR, or socket/ROS/LCM motion sends.

## M3-A0 Autonomous SLAM Usage

M3-A0 adds `AutonomousSlamCoordinator` as the hardware-ready autonomous SLAM state machine. It uses the algorithm motion API through `RobotSlamMotionPort`, so autonomous mapping logic can request stop or active-scan rotation without knowing the software-side chassis implementation. The app still does not enable live motion by default. See `docs/HARDWARE_READY_AUTONOMOUS_SLAM_CORE.md`.
