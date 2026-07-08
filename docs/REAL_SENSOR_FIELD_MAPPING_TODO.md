# Real Sensor Field Mapping TODO

## Sensor Owner

- Confirm ToF `request_start_s` source.
- Confirm ToF `response_received_s` source.
- Confirm ToF `estimated_sample_time_s` estimate.
- Confirm ToF `request_latency_s` calculation.
- Confirm ToF `frame_id`.
- Confirm ToF range unit.
- Confirm ToF `angle_min_rad`, `angle_max_rad`, and `angle_increment_rad`.
- Confirm ToF `range_min_m` and `range_max_m`.
- Confirm IMU timestamp source and unit.
- Confirm IMU yaw-rate source and unit.
- Confirm IMU acceleration source and unit.
- Confirm Wheel `request_start_s` source.
- Confirm Wheel `response_received_s` source.
- Confirm Wheel `estimated_sample_time_s` estimate.
- Confirm Wheel `request_latency_s` calculation.
- Confirm Wheel linear velocity source and unit.
- Confirm Wheel angular velocity source and unit.
- Confirm Wheel valid flag source.
- Confirm multi-sensor sync plan.

## Software Owner

- Confirm data frequency.
- Confirm data buffering.
- Confirm missing-frame policy.
- Confirm request-window recording.
- Confirm time source.

## Project Manager

- Assign sensor data provider.
- Schedule field alignment meeting.
- Reserve real data capture window.
- Assign acceptance owner.
- Confirm M3-B1 acceptance before lifted tests.

## M3-B2 Log Fields

Future capture tooling must record ToF/Wheel request start, response receipt, midpoint estimate, and request latency. A single timestamp is insufficient for replay.
