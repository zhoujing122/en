# Multi-ToF Replay Log Contract

M3-C3 implements an in-memory 3-ToF replay/log pipeline. Text lines are parsed into `MultiTofRawPacket`, passed through `MultiTofSyncChecker`, then through `MultiTofSnapshotBuilder`, and successful replay returns `RobotSlamSensorSnapshot.has_multi_tof=true`.

The codec is strict. Parse errors are invalid records, not comments. Invalid numeric fields, invalid bool fields, missing fields, and empty ranges are rejected with line number and raw line preserved.

The log preserves request-window timing for front, left, right, and wheel: request start, response received, estimated sample time, and request latency.

This stage only processes strings and vectors of strings. No real replay file is read, no real hardware is read, and no full autonomous fake pipeline is started. M3-C4 is the next stage for 3-ToF fake/replay full pipeline integration.
