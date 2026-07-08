# Real Sensor Adapter Data Contract

M3-B1 defines the real sensor adapter data contract skeleton. It does not
implement real ToF, IMU, or Wheel drivers. It does not read hardware, use SDKs,
or connect ROS, LCM, socket, or serial paths.

ToF and Wheel timing is request-based. Their timestamps are request-window
estimates, not hardware capture timestamps. The request timing fields are:

- `request_start_s`: time when the read request starts.
- `response_received_s`: time when the response is received.
- `estimated_sample_time_s`: midpoint estimate of the request window.
- `request_latency_s`: response time minus request start time.
- `timing_source`: source label for the timing estimate.

`RealSensorRawTofFrame` carries frame id, ranges in meters, angles in radians,
range limits in meters, source, sequence, and request timing.

`RealSensorRawImuSample` carries timestamp, frame id, yaw rate in rad/s,
acceleration in m/s^2, source, and sequence. IMU may keep direct timestamp
semantics because it can be a continuous stream.

`RealSensorRawWheelSample` carries frame id, valid flag, linear velocity in
m/s, angular velocity in rad/s, source, sequence, and request timing.

`RealSensorRawPacket` combines ToF, IMU, and Wheel data before conversion to
`RobotSlamSensorSnapshot`.

`RealSensorDataContractChecker` validates packet age, required sensors, ToF
range data, IMU values, Wheel values, request latency, and multi-sensor sync.
ToF and Wheel sync uses `estimated_sample_time_s`.

`RealSensorSnapshotBuilder` converts a valid raw packet into
`RobotSlamSensorSnapshot`. Future `RealTofImuWheelSensorPort` implementations
should call the builder rather than bypassing the contract.

Passing this acceptance does not prove real sensor readiness.

## M3-B2 Timing Hardening

M3-B2 rejects negative or mismatched request latency, estimated sample times outside the request window, estimated times that are not close to the midpoint, future sensor times, and packet-to-sensor time mismatches. ToF/Wheel remain request-window estimates, not hardware capture timestamps. See `REAL_SENSOR_REPLAY_LOG_FORMAT.md`.
