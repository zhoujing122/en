# Multi-ToF Raw Data Contract

M3-C0 defines the raw contract for the robot three-ToF layout. The current runtime still uses the existing single-ToF `RealSensorRawPacket` and single `RobotSlamSensorSnapshot`; this stage only adds offline contract types and checks for future adapters.

The required mounts are:

- `tof_front_frame`: front ToF, yaw `0` radians.
- `tof_left_frame`: left ToF, yaw `+90` degrees (`+pi/2`).
- `tof_right_frame`: right ToF, yaw `-90` degrees (`-pi/2`).

Each ToF keeps request-window timing fields: `request_start_s`, `response_received_s`, `estimated_sample_time_s`, and `request_latency_s`. The estimated sample time remains a request-window midpoint estimate, not a hardware capture timestamp.

M3-C0 does not perform three-ToF pairwise sync, IMU/Wheel sync, multi-ToF snapshot building, replay/log conversion, or hardware access. Those are intentionally deferred to M3-C1, M3-C2, and M3-C3.

## M3-C1 Sync Layer

The raw contract is now consumed by the M3-C1 sync checker. Raw validation remains separate: M3-C0 verifies frame/timing/range/mount correctness, while M3-C1 verifies front/left/right pairwise sync plus IMU/Wheel alignment. Snapshot building is still not implemented here.
