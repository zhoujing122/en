# Real Adapter Implementation TODO

## Sensor Owner

- Define ToF data field sources.
- Define IMU data field sources.
- Define wheel odometry data field sources.
- Confirm timestamp unit and time base.
- Confirm coordinate frames.
- Confirm data rates.

## Software Chassis Owner

- Implement STOP.
- Implement E-STOP.
- Implement TTL expiry.
- Report accepted and rejected acknowledgements.
- Confirm TURN_LEFT and TURN_RIGHT direction.
- Map `speed_normalized` to the approved low-speed command range.

## SLAM and Map Owner

- Implement the backend update API.
- Produce `RobotSlamMapQuality`.
- Implement map save behavior.
- Report update latency.
- Map backend fault codes to `SlamBackendFault`.

## Project Manager

- Assign owners.
- Set schedule.
- Reserve vehicle test window.
- Confirm lifted test conditions.
- Run acceptance review before live motion.

## M3-B1 Sensor Field Mapping

The sensor owner must map ToF and Wheel request-window timing fields before live work. Use `docs/REAL_SENSOR_FIELD_MAPPING_TODO.md` for the required field checklist.

## M3-B2 Sensor Replay TODO

Before live integration, real capture tooling should emit the M3-B2 replay log format so sensor, SLAM, and policy regressions can run offline.

## M3-B3 SLAM Backend TODO

Future real backend work should replace `DeterministicSlamBackendBinding` with a production `SlamBackendBinding` implementation after replay-to-backend regression and map quality acceptance pass.

M3-B4 replacement points: RealSensorReplayPort -> RealTofImuWheelSensorPort, FullAutonomousSlamFakeMotionPort -> RealSoftwareMotionPort, DeterministicSlamBackendBinding -> RealSlamBackendBinding.

## M3-B5 Follow-Up

Future real adapters should preserve the phase-aware replay lessons: motion command and settle phases must not silently consume unrelated sensor records, and real map storage must replace fake metadata only after explicit readiness review.

## M3-B6 Relocalization TODO

Future real adapters must keep fake relocalization separate from real pose writeback until a real map backend, scan matcher, confidence policy, and safety gate are available.
