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
