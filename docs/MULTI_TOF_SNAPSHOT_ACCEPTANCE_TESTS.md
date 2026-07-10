# Multi-ToF Snapshot Acceptance Tests

M3-C2 acceptance covers:

- valid synchronized packet builds `has_multi_tof`.
- front, left, and right frames are preserved.
- `synchronized_time_s` matches the sync checker output.
- IMU timestamp is preserved.
- Wheel timestamp uses request-window estimated sample time.
- degraded two-ToF mode can build when sync degradation allows it.
- invalid sync is rejected.
- legacy primary front, median-time-closest, and first-present modes are covered.
- report output includes no-real-hardware and no-replay/log disclaimers.
