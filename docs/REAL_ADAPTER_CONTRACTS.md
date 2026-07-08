# Real Adapter Contracts

M3-A1 defines the real adapter contract. It does not implement real drivers.
Future adapters must translate real robot data into these project structures:

- `TofScanFrame`
- `ImuFrame`
- `WheelOdomFrame`
- `RobotSlamSensorSnapshot`
- `RobotSlamMapQuality`

ToF contract:

- timestamp must be finite and recent
- range count must be within configured bounds
- `angle_increment_rad` must be finite and positive
- `max_range_m` must be finite and positive
- finite ranges must be within configured min/max range
- NaN ratio must stay below the configured limit
- frame id is required by default

IMU contract:

- timestamp must be finite and recent
- yaw rate must be finite
- acceleration values must be finite
- yaw rate and acceleration must stay within configured limits

Wheel contract:

- valid flag must be true
- timestamp must be finite and recent
- linear and angular motion must be finite
- linear and angular motion must stay within configured limits

Map contract:

- coverage ratio must be finite and in [0,1]
- yaw coverage ratio must be finite and in [0,1]
- valid scan count must be non-negative
- poor quality should provide a reason

Real adapters should pass `RealAdapterContractChecker` before their data reaches
`AutonomousSlamCoordinator`. This stage does not connect ROS, device files,
network senders, SDKs, or real map backends.

M3-A2 consumes these contracts through docs/PRELIVE_AUTONOMOUS_SLAM_INTEGRATION.md before any lifted low-speed direction probe work.

M3-A3 separates real sensor adapter contracts from the SLAM backend binding contract in docs/SLAM_BACKEND_MAP_PORT_BINDING.md.
M3-A4 adds `docs/AUTONOMOUS_SLAM_E2E_PRELIVE_SCENARIO.md` for the end-to-end pre-live shadow scenario across replay sensors, SLAM backend binding, map port, pre-live runner, coordinator, policy, and shadow motion.

## M3-B0 Stub Handoff

M3-B0 adds the real adapter stub classes that future implementation owners should fill after the M3-A1 contracts are satisfied. The stubs remain not ready and do not validate real hardware.

## M3-B1 Sensor Raw Packet Contract

M3-B1 defines `RealSensorRawPacket` and `RealSensorSnapshotBuilder` for future sensor adapters. ToF and Wheel timing is request-based and must not be treated as hardware capture timestamps.
