# Software Direction-Speed Motion Command Interface

This document defines the contract between the robot_slamd algorithm layer and
the software chassis-control layer.

## Boundary

The algorithm layer does not write motors directly. It does not write BL4820
registers, emit UART speed commands, generate PWM/FR output, or encode the
low-level motor protocol. The software chassis-control layer must implement the
direction+speed interface described here.

The algorithm layer may call this interface only after MotionSafetyExecutor and
MotionWriteController allow dispatch. Command stale checks, deadman, duration
latch, encoder feedback gates, obstacle checks, current/status checks, and zero
command sequencing remain upstream safety requirements.

M2-B0 is interface scaffolding only. The production path remains fail-closed.

## Directions

The software side must support:

- STOP
- TURN_LEFT
- TURN_RIGHT
- FORWARD
- BACKWARD
- EMERGENCY_STOP

For M2-B0/M2-B1 active scan work, only these directions may be used:

- STOP
- TURN_LEFT
- TURN_RIGHT
- EMERGENCY_STOP

FORWARD and BACKWARD are defined for the contract but remain disabled by
default. They must not be enabled for active scan writeback until a later stage
explicitly gates translation commands.

## Speed

Commands use normalized speed in the range [0.0, 1.0]. The software chassis
layer maps this normalized value to physical chassis speed, wheel RPM, or motor
gear level. The algorithm layer must cap the maximum normalized speed in config;
the default is intentionally small.

## Lifetime

Every command includes:

- timestamp_s
- ttl_s
- sequence
- source
- reason

The software side must treat direction+speed commands as expiring commands, not
permanent motion requests. If no fresh command is received before ttl_s expires,
the software side must stop the chassis. STOP must be idempotent and always
safe to repeat. EMERGENCY_STOP has the highest priority.

## Acknowledgement

The software side must return accepted/rejected status. M2-B0 provides only a
FakeSoftwareMotionCommandTransport for tests. Production code must not attach a
real transport in this stage.

Any transport failure, rejected command, stale command, safety fault, or invalid
command must result in STOP as the preferred next action.

## Algorithm Motion API

The high-level algorithm command facade is documented in `docs/ALGORITHM_MOTION_API.md`. New algorithm modules should express stop, emergency stop, direction probe, active-scan turn, and recovery intent through that API, then convert to `SoftwareMotionCommand` for software-side transport. Software-side code should consume direction + speed + duration + TTL, not algorithm-internal wheel RPM.
