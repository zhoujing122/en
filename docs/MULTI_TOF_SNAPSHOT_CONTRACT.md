# Multi-ToF Snapshot Contract

M3-C2 converts a synchronized `MultiTofRawPacket` into `RobotSlamSensorSnapshot.has_multi_tof`.

The snapshot keeps front, left, and right `TofScanFrame` entries with their own frame ids, ranges, and request-window estimated timestamps. `synchronized_time_s` comes directly from `MultiTofSyncChecker` and is normally the median present ToF time.

IMU is copied with `imu.timestamp_s`. Wheel odometry uses the request-window `estimated_sample_time_s` as its snapshot timestamp.

Legacy single-ToF compatibility remains enabled by default. `snapshot.has_tof` and `snapshot.tof` are populated from the configured legacy primary, defaulting to front. Existing code that only reads `has_tof` continues to work.

This stage does not implement Multi-ToF replay/log. It does not read real hardware and does not change live SLAM, mapping, localization, or motion behavior.
