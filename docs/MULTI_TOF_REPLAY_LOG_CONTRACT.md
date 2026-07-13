# Multi-ToF Replay Log Contract

M3-C3 implements an in-memory 3-ToF replay/log pipeline. Text lines are parsed into `MultiTofRawPacket`, passed through `MultiTofSyncChecker`, then through `MultiTofSnapshotBuilder`, and successful replay returns `RobotSlamSensorSnapshot.has_multi_tof=true`.

The codec is strict. Parse errors are invalid records, not comments. Invalid numeric fields, invalid bool fields, missing fields, and empty ranges are rejected with line number and raw line preserved.

The log preserves request-window timing for front, left, right, and wheel: request start, response received, estimated sample time, and request latency.

This stage only processes strings and vectors of strings. No real replay file is read, no real hardware is read, and no full autonomous fake pipeline is started. M3-C4 is the next stage for 3-ToF fake/replay full pipeline integration.

## M3-C3.1 Protocol Alignment Note

M3-C3.1 replaces the M3-C three-ToF replay contract with scalar single-point ToF readings backed by the confirmed 9-byte payload: 48-bit echo tag, `distance_mm`, and `confidence`. The echo tag is retained for correlation/debug only and is not used as measurement time. Synchronization uses request-window midpoint timestamps. The default full ToF FOV is 1.6 degrees and must not be expanded into synthetic ranges or point clouds. Legacy `snapshot.tof` is only a one-range compatibility projection for old fake backends. This stage does not connect real ToF/UART hardware, enable real motion, write real maps, perform pose writeback, or implement true three-ToF SLAM fusion.
