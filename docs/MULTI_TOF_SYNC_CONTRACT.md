# Multi-ToF Sync Contract

M3-C1 implements the offline synchronization checker for three ToF frames plus IMU and Wheel samples. It consumes the M3-C0 `MultiTofRawPacket` and does not build a `RobotSlamSensorSnapshot`.

Effective time policy:

- front ToF uses `front.timing.estimated_sample_time_s`.
- left ToF uses `left.timing.estimated_sample_time_s`.
- right ToF uses `right.timing.estimated_sample_time_s`.
- IMU uses `imu.timestamp_s`.
- Wheel uses `wheel.timing.estimated_sample_time_s`.

The default `multi_tof_time` is `median_present_tof`, computed from present ToF estimated sample times. The checker also supports `front` and `mean_present_tof` references for explicit tests and future staged policies.

Sync checks are fail-closed:

- front-left, front-right, and left-right pairwise ToF sync default threshold: 50 ms.
- multi-ToF to IMU default threshold: 100 ms.
- multi-ToF to Wheel default threshold: 100 ms.

This stage does not implement the M3-C2 multi-ToF snapshot builder, replay/log support, hardware drivers, or app runtime startup.
