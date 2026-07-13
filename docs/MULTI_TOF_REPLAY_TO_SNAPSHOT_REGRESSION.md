# Multi-ToF Replay To Snapshot Regression

The regression path is:

`text log -> MultiTofRawPacket -> MultiTofSyncChecker -> MultiTofSnapshotBuilder -> RobotSlamSensorSnapshot`.

The regression verifies front, left, and right frame ids, `synchronized_time_s`, IMU timestamp, Wheel request-window timestamp, and legacy `has_tof` compatibility.

It also verifies sync failure logs do not produce a reused previous snapshot.

## M3-C3.1 Protocol Alignment Note

M3-C3.1 replaces the M3-C three-ToF replay contract with scalar single-point ToF readings backed by the confirmed 9-byte payload: 48-bit echo tag, `distance_mm`, and `confidence`. The echo tag is retained for correlation/debug only and is not used as measurement time. Synchronization uses request-window midpoint timestamps. The default full ToF FOV is 1.6 degrees and must not be expanded into synthetic ranges or point clouds. Legacy `snapshot.tof` is only a one-range compatibility projection for old fake backends. This stage does not connect real ToF/UART hardware, enable real motion, write real maps, perform pose writeback, or implement true three-ToF SLAM fusion.
