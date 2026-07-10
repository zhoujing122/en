# Multi-ToF Replay Acceptance Tests

M3-C3 acceptance covers:

- valid log decode.
- replay port readiness.
- valid replay producing `has_multi_tof` snapshot.
- two-packet consumption.
- invalid numeric rejection.
- missing field rejection.
- empty ranges rejection.
- pairwise sync failure rejection.
- IMU sync failure rejection.
- Wheel sync failure rejection.
- degraded missing-left replay passing when `AllowMissingOne` is configured.
