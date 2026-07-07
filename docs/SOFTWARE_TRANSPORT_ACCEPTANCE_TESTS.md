# Software Transport Acceptance Tests

These checks define the minimum software-side transport acceptance contract before live motion.

## STOP

1. Single STOP is accepted.
2. Repeated STOP is accepted.
3. STOP does not depend on `enable_live_motion`.
4. STOP does not produce motion.
5. STOP can interrupt non-zero commands from any state.

## EMERGENCY_STOP

1. EMERGENCY_STOP is accepted.
2. EMERGENCY_STOP has highest priority.
3. After EMERGENCY_STOP, non-zero commands are rejected until software side reset or fault clear.
4. EMERGENCY_STOP does not produce motion.

## TURN_LEFT / TURN_RIGHT

1. Shadow mode accepts commands but does not move hardware.
2. Live testing requires lifted wheels.
3. Direction convention must be recorded.
4. Unconfirmed direction convention forbids grounded motion.

## FORWARD / BACKWARD

1. Disabled by default.
2. Disabled translation commands must be rejected.
3. Translation can only be enabled after a later low-speed grounded test phase.

## TTL

1. `ttl_s` timeout must automatically STOP.
2. Fresh commands refresh ttl.
3. If the algorithm process stalls, xiaoen must stop automatically.
4. `ttl_s <= 0` is rejected.

## ACK

1. `ok=false` means send failure.
2. `ok=true, accepted=false` means rejected.
3. Only `accepted=true` means execution was accepted.
4. Rejected commands must include an error string.

## Shadow

1. Shadow records commands only.
2. Shadow does not touch hardware.
3. Shadow can replay command sequences.
4. Shadow validates STOP, TTL fields, ACK, and E-STOP semantics.

## Live Gate

1. `enable_live_motion` defaults false.
2. Robot lifted or wheels free must be confirmed.
3. Emergency stop must be available.
4. Operator must be present.
5. First direction probe speed <= `0.03`.
6. First direction probe duration <= `0.30s`.
