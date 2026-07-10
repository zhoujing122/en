# Multi-ToF Replay To Snapshot Regression

The regression path is:

`text log -> MultiTofRawPacket -> MultiTofSyncChecker -> MultiTofSnapshotBuilder -> RobotSlamSensorSnapshot`.

The regression verifies front, left, and right frame ids, `synchronized_time_s`, IMU timestamp, Wheel request-window timestamp, and legacy `has_tof` compatibility.

It also verifies sync failure logs do not produce a reused previous snapshot.
