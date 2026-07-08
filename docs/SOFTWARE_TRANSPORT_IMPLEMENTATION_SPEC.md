# Software Transport Implementation Spec

This is the implementation spec for the software-side chassis transport used by xiaoen robot_slamd.
The algorithm layer does not write motors, BL4820 registers, UART speed commands, PWM, or FR outputs.
The software side must implement `SoftwareMotionCommandTransport` or an equivalent adapter.

## Command Contract

The software side receives `SoftwareMotionCommand` fields:

- `direction`
- `speed_normalized`
- `timestamp_s`
- `ttl_s`
- `source`
- `reason`
- `sequence`

Required directions for M2-B1/M2-B3:

- `STOP`
- `TURN_LEFT`
- `TURN_RIGHT`
- `EMERGENCY_STOP`

`FORWARD` and `BACKWARD` are defined for future use, but remain disabled by default. Software may reserve implementation hooks, but must not enable translation by default.

## Speed And Timing

`speed_normalized` is in `[0, 1]`. The software chassis layer maps it to actual chassis speed, motor RPM, or internal control levels.
Recommended M2-B1/M2-B3 caps:

- direction probe speed <= `0.03`
- active scan speed <= `0.05`
- command duration <= `0.50s`

`timestamp_s` is used for timing checks. `ttl_s` is mandatory stop timing, not a suggestion. If no fresh command is received before `ttl_s` expires, the software side must automatically issue STOP.

## STOP And Emergency Stop

STOP must be idempotent. Repeated STOP commands must be accepted and safe. `EMERGENCY_STOP` has the highest priority and must interrupt any non-STOP command.

## Send Result Contract

The software side returns `SoftwareMotionSendResult`:

- `ok`
- `accepted`
- `error`
- `transport_sequence`
- `timestamp_s`

`ok=false` means transport send failed. `ok=true, accepted=false` means the transport processed the command but rejected execution. Rejected commands must provide a stable `error` string.

## Rejection Requirements

The software side must reject invalid speed, invalid ttl, old timestamp, disabled translation, non-shadow restriction violations, and any command rejected by its live enable gate.

## Shadow And Live Gates

The software side must provide a shadow/dry-run mode that records commands and does not move hardware. It must also provide a real motion enable switch, default false. Shadow acceptance tests must pass before live testing. Lifted direction probe must pass before any grounded motion.

No command is permanent. Every command is time-limited. Any abnormal condition must result in STOP.

## M3-A0 Port Integration

The future real software transport should be wrapped by a `RobotSlamMotionPort` adapter for autonomous SLAM. The coordinator only emits algorithm-level stop and active-scan rotation commands in M3-A0; real TTL stop, accepted/rejected semantics, and live enable gates remain the responsibility of the software-side chassis layer. See `docs/HARDWARE_READY_AUTONOMOUS_SLAM_CORE.md`.
