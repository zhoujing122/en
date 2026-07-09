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

## M3-B2.1 Robustness Acceptance

Additional acceptance checks require invalid lines to become InvalidRecord, comment-only logs to be rejected, invalid numeric fields to fail parsing, and RealSensorReplayPort to return an empty snapshot when fail_on_contract_error is true. The default time mode is record_packet_time so recorded logs are not judged by an unrelated external clock.
