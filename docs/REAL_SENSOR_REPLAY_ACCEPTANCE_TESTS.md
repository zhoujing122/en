# Real Sensor Replay Acceptance Tests

M3-B2 replay acceptance checks:

1. Valid log parse passes.
2. Valid log replay passes through `RealSensorReplayPort`.
3. Request latency too high log fails.
4. Sync too large log fails.
5. Replay port ready check is enforced.
6. No hardware access is performed.
7. No motion command is emitted.
8. ToF/Wheel request-window timestamp fields are retained.

Passing these tests validates the offline replay/log contract only. It does not prove real ToF/IMU/Wheel readiness.
