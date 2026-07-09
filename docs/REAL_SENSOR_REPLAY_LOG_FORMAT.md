# Real Sensor Replay Log Format

M3-B2 defines the offline replay/log contract for future real sensor captures. It does not read real hardware, does not default to file reads, and does not start a replay runner from app runtime.

Each packet line is comma-separated `key=value` text. A packet must preserve the full ToF and Wheel request window: `tof_req_start`, `tof_resp`, `tof_est`, `tof_latency`, `wheel_req_start`, `wheel_resp`, `wheel_est`, and `wheel_latency`. A log that only stores one ToF or Wheel timestamp is not valid for this contract.

IMU can use `imu_time` because IMU may be a continuous stream. ToF and Wheel timestamps are request-window estimates, not hardware capture timestamps. The default estimate is the midpoint of request start and response receipt.

`RealSensorReplayPort` parses replay records through `RealSensorSnapshotBuilder`, producing `RobotSlamSensorSnapshot` without touching hardware. Replay is for future regression testing of sensor conversion, SLAM backend binding, map quality, and policy behavior. Replay passing does not mean live hardware is ready.

## M3-B2.1 Robust Parsing

M3-B2.1 requires parse errors to remain InvalidRecord records instead of comments. Invalid numeric fields, invalid bool fields, missing required fields, and empty tof_ranges are rejected during parsing. Replay logs still must retain ToF/Wheel request-window timing fields and must not degrade to a single timestamp. Default replay validation uses record_packet_time.
