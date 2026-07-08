# Real Adapter Stubs

M3-B0 adds real adapter stubs. These files are not real hardware
implementations and are not live motion paths.

The stubs reserve the exact classes that future vehicle integration should
fill or replace:

- `RealTofImuWheelSensorPortStub` implements `RobotSlamSensorPort`.
- `RealSoftwareMotionPortStub` implements `RobotSlamMotionPort`.
- `RealSlamBackendBindingStub` implements `SlamBackendBinding`.
- `RealAdapterFactory` creates only disabled stubs in this phase.

All stubs default to not ready. They do not read sensors, do not send motion,
do not update a real map, and do not save a real map.

Passing the M3-B0 tests only proves the handoff skeleton is present and
fail-closed. It does not prove hardware readiness.

Future work should fill the real adapter files so that the existing
`AutonomousSlamCoordinator`, policy, pre-live runner, and E2E runner do not
need to change.

## M3-B1 Sensor Data Contract

M3-B1 adds the real sensor data contract skeleton. Future real sensor adapters should fill `RealSensorRawPacket` and call `RealSensorSnapshotBuilder`. ToF and Wheel timestamps are request-window estimates, not hardware capture timestamps.
