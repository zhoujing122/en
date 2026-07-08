# Real Sensor Adapter Acceptance Tests

M3-B1 acceptance covers the data contract skeleton only.

- Valid packet passes.
- Missing ToF fails.
- Missing IMU and Wheel fails.
- Empty ToF ranges fail.
- Invalid ToF angle fails.
- High ToF NaN ratio fails.
- Invalid IMU fails.
- Invalid Wheel fails.
- Invalid request timing fails.
- Request latency too high fails.
- Sync too large fails.
- Snapshot build passes.
- No real hardware is read.
- No real motion is triggered.

ToF and Wheel timestamps are request-window estimates, not hardware capture
timestamps.

## M3-B2 Replay Coverage

M3-B2 adds offline replay acceptance for valid logs, request latency failures, and sync failures while preserving ToF/Wheel request-window fields. See `REAL_SENSOR_REPLAY_ACCEPTANCE_TESTS.md`.
