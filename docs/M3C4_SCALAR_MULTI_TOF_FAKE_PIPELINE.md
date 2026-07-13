# M3-C4 Scalar Multi-ToF Fake Pipeline

## Goal

M3-C4 connects the M3-C3.1 scalar three-ToF replay path to the existing full fake autonomous SLAM pipeline.

The data path is:

```text
9-byte scalar three-ToF replay log
-> MultiTofReplayLogCodec
-> scalar raw ToF frame
-> contract checker
-> sync checker
-> RobotSlamSensorSnapshot
-> MultiTofReplaySensorPort
-> FullAutonomousSlamRunner
-> AutonomousSlamCoordinator
-> legacy scalar ToF projection
-> DeterministicSlamBackendBinding
-> FullAutonomousSlamFakeMotionPort
-> fake pipeline report
```

## C3.1 Prerequisites

M3-C3.1 is the required input contract for this stage:

- each ToF is a single scalar distance reading, not a scan;
- replay payloads use the confirmed 9-byte payload;
- `echo_tag_u48` is preserved but is not measurement time;
- effective measurement time is the request/response midpoint;
- protocol range is 20 mm to 12000 mm;
- mapping range is 50 mm to 12000 mm;
- confidence 0 is invalid, 1-69 is diagnostic only, and 70-100 is mapping usable;
- default full FOV is 1.6 degrees and is not expanded into beams.

## RobotSlamSensorPort Adapter

`MultiTofReplaySensorPort` wraps `MultiTofReplayPort` and implements `RobotSlamSensorPort`.

`ready()` means the replay input is constructable and has packet data. Replay completion is reported separately through `replay_completed()` and does not become an initialization failure.

`latest_snapshot()` consumes at most one packet per call. Comments can be skipped because they are not packets. Invalid records, sync failures, contract failures, snapshot build failures, and EndOfReplay return a default empty `RobotSlamSensorSnapshot`; the adapter does not return or retime a previous successful snapshot.

The adapter exposes read counters and the last `MultiTofReplayReadResult` so tests and reports can distinguish invalid payloads, sync failures, rejected snapshots, and EndOfReplay.

## Runner Injection

`FullAutonomousSlamRunner::run_once()` still builds the original `RealSensorReplayPort` and preserves legacy behavior. M3-C4 adds `run_once_with_sensor()` so callers can inject any `RobotSlamSensorPort` implementation.

The runner still constructs the same deterministic backend, map adapter, and fake motion port. It does not include or special-case `MultiTofReplaySensorPort`.

## Coordinator Payload Detection

The coordinator now treats `has_multi_tof` as an explicit sensor payload. If an input snapshot already contains only `has_multi_tof=true`, the coordinator does not call `sensor_port->latest_snapshot()` to consume the next replay packet. Initialization may still fail if the deterministic backend path requires legacy `has_tof`; M3-C4 keeps that failure explicit.

Completely empty snapshots keep the previous behavior and can still be filled from the configured sensor port.

## Phase-Aware Sensor Consumption

The full pipeline keeps the existing phase-aware read policy. Sensor reads are allowed during startup, WaitingForSensors, Initializing, Mapping, and IntegratingScan. Reads are skipped during NeedActiveScan, SendingMotionCommand, WaitingMotionSettle, Completed, and Fault.

M3-C4 tests verify that replay cursor/read counts do not advance while fake motion is being sent or settled.

## Error Propagation

Replay and snapshot errors are exposed before backend integration:

- non-9-byte payloads;
- invalid hex;
- distance outside protocol range;
- confidence outside contract;
- missing required ToF in RequireAll mode;
- pairwise ToF sync failure;
- ToF/IMU sync failure;
- ToF/Wheel sync failure;
- EndOfReplay.

On failure, the adapter returns an empty snapshot, keeps the concrete replay fault, and does not reuse the previous success.

## EndOfReplay

EndOfReplay is deterministic. It returns an empty snapshot, increments the adapter end counter, preserves the last successful measurement timestamp unchanged, and does not create an extra backend update or fake motion command.

## AllowMissingOne

The M3-C4 degraded scenario is fixed to missing right. Front and left remain present, so front can still provide the legacy scalar projection. RequireAll with the same missing-right input is rejected before backend integration.

## Legacy Scalar Projection

The deterministic backend still consumes `snapshot.tof`. M3-C4 verifies that the compatibility projection:

- contains at most one range;
- uses the front ToF when available and mapping usable;
- carries `legacy_scalar_tof_projection` in its source;
- uses the front effective timestamp;
- does not synthesize a scan fan, point cloud, or multiple beams.

## Backend Limitation

M3-C4 proves that scalar three-ToF data reaches the full fake pipeline. It does not implement native three-ToF map fusion. `native_multi_tof_backend_consumption=false` remains an explicit report field.

Current backend behavior is:

```text
multi-ToF snapshot present
legacy front scalar projection present
DeterministicSlamBackendBinding consumes snapshot.tof only
```

## Fake Motion And Safety

M3-C4 uses `FullAutonomousSlamFakeMotionPort` only. It does not enable real motion, real `/dev` access, UART, I2C, ioctl, motor writers, map writes, pose writeback, or relocalization writeback. Forward and Backward remain disallowed in the fake autonomous pipeline acceptance path.

## Tests

The M3-C4 tests cover:

- sensor adapter ready, counters, status, EndOfReplay, invalid record, and no stale reuse;
- normal RequireAll full pipeline;
- ActiveScanRequired phase-aware replay consumption;
- AllowMissingRight degraded success;
- MissingRightRequireAll rejection;
- LowConfidenceFront rejection from legacy mapping;
- invalid payload length and invalid hex;
- 12001 mm distance rejection;
- pairwise, IMU, and wheel sync rejection;
- backend rejection;
- nullptr and not-ready injected sensors;
- report writer wording;
- coordinator multi-ToF-only no-double-read regression.

## Next Stage: M3-D0

M3-D0 should introduce the closed-loop fake simulation entry points:

- `FakeWorld`
- `SimRobotPlant`
- `SimMotionPort`
- `SimWheelEncoder`
- `SimImu`
- `SimThreeScalarTof`

M3-C4 does not implement those components.
