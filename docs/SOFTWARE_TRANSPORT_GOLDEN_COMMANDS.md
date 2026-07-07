# Software Transport Golden Commands

These examples are the canonical direction + speed commands for software-side transport implementation.

## STOP

- direction: `STOP`
- speed_normalized: `0.0`
- ttl_s: `0.30`
- source: `SafetyStop`
- reason: `safety_stop`

## EMERGENCY_STOP

- direction: `EMERGENCY_STOP`
- speed_normalized: `0.0`
- ttl_s: `0.30`
- source: `SafetyStop`
- reason: `emergency_stop`

## DIRECTION_PROBE_LEFT

- direction: `TURN_LEFT`
- speed_normalized: `0.03`
- duration_s: `0.30`
- ttl_s: `0.30`
- source: `ManualTest`
- reason: `direction_probe_left`

## DIRECTION_PROBE_RIGHT

- direction: `TURN_RIGHT`
- speed_normalized: `0.03`
- duration_s: `0.30`
- ttl_s: `0.30`
- source: `ManualTest`
- reason: `direction_probe_right`

## ACTIVE_SCAN_TURN_LEFT

- direction: `TURN_LEFT`
- speed_normalized: `0.05`
- duration_s: `0.50`
- ttl_s: `0.30`
- source: `ActiveScan`
- reason: `active_scan_turn_left`

## ACTIVE_SCAN_TURN_RIGHT

- direction: `TURN_RIGHT`
- speed_normalized: `0.05`
- duration_s: `0.50`
- ttl_s: `0.30`
- source: `ActiveScan`
- reason: `active_scan_turn_right`

## FORWARD Disabled Example

- direction: `FORWARD`
- speed_normalized: `0.03`
- expected: rejected until translation is enabled and low-speed ground test has passed.

## BACKWARD Disabled Example

- direction: `BACKWARD`
- speed_normalized: `0.03`
- expected: rejected until translation is enabled and low-speed ground test has passed.
