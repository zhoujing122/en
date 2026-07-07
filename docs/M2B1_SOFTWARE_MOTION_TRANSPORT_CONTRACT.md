# M2-B1 Software Motion Transport Contract

This contract is for Xiaoen `robot_slamd` / `en`. It is not a machine dog interface.

The algorithm layer defines `SoftwareMotionCommand` with these fields:

- `direction`
- `speed_normalized`
- `timestamp_s`
- `ttl_s`
- `source`
- `reason`
- `sequence`

The software-side production transport must receive all fields. For M2-B1-live it must support only:

- `STOP`
- `TURN_LEFT`
- `TURN_RIGHT`
- `EMERGENCY_STOP`

`FORWARD` and `BACKWARD` are defined by the API but disabled for M2-B1.

`speed_normalized` is in `[0, 1]`. For the first lifted direction probe the maximum must be `<= 0.03`. After direction convention is confirmed, lifted live checks may use up to `0.05`. Code defaults must not exceed `0.05` unless an operator deliberately edits configuration.

The software side must enforce `ttl_s`: if a fresh command is not received before TTL expiry, it must automatically stop. `STOP` must be idempotent and safe to repeat. `EMERGENCY_STOP` has the highest priority.

The transport return must include:

- `ok`
- `accepted`
- rejected reason
- transport sequence
- timestamp

If transport send fails or is rejected, the algorithm-side writer must enter the writer-fault latch and the next action must prioritize STOP. The software side must not treat commands as permanent motion requests.

Before any live motion, software must provide a shadow/dry-run mode that accepts commands but does not move the chassis, plus a real-motion enable switch that defaults false. External or operator emergency-stop procedure must be documented.

M2-B1 uses two lifted-only preflight modes:

- `LiftedDirectionProbe`: first lifted low-speed direction verification. Direction convention may be pending. It is limited to `speed_normalized <= 0.03` and `duration_s <= 0.30`.
- `ConfirmedLiftedLive`: lifted low-speed motion after direction convention is verified. Direction convention must be confirmed before this mode can pass.

The current algorithm convention is left wheel negative and right wheel positive means `TURN_LEFT`; left positive and right negative means `TURN_RIGHT`. This must be verified during `LiftedDirectionProbe` before any `ConfirmedLiftedLive` run or later ground motion. If reversed, adjust the software mapping/configuration first; do not compensate by unsafe hardware changes.
