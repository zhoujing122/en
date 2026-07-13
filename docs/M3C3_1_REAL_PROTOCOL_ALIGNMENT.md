# M3-C3.1 Real Protocol Alignment

M3-C3.1 aligns the fake/replay sensor contracts with the currently known real ToF and BL4820 protocol facts. It does not connect real ToF, real UART, real IMU, real chassis motion, real map writing, pose writeback, frontier exploration, A*, or production SLAM.

## Sensor Set

The target robot uses three independent single-point ToF sensors (`front`, `left`, `right`), IMU, and two BL4820 wheel encoders on independent UARTs. Each ToF produces one distance and one confidence value per read. It is not a 2D lidar, does not produce a point cloud, and must not be expanded into synthetic beams.

## ToF Payload

The currently confirmed minimum ToF payload is exactly 9 bytes:

- bytes 0-5: little-endian 48-bit echo tag
- bytes 6-7: little-endian `distance_mm`
- byte 8: `confidence`

Golden sample:

`11 22 33 44 00 00 AB 0A 64`

Decoding:

- `echo_tag_u48 = 0x000044332211 = 1144201745`
- `distance_mm = 0x0AAB = 2731`
- `distance_m = 2.731`
- `confidence = 100`

The echo tag is a request correlation value. It is retained in replay, snapshot, and reports, but it is not a sensor hardware timestamp, monotonic time, Unix time, microsecond count, millisecond count, or SLAM measurement time.

## ToF Validity

Protocol range:

- `tof_protocol_min_range_m = 0.02`
- `tof_protocol_max_range_m = 12.0`

Mapping candidate range:

- `tof_mapping_min_range_m = 0.05`
- `tof_mapping_max_range_m = 12.0`
- `tof_mapping_min_confidence = 70`

Rules:

- `<20mm`: protocol invalid
- `20..12000mm`: protocol distance valid
- `>12000mm`: protocol invalid
- `confidence == 0`: invalid measurement, not usable for occupied or free-space updates
- `confidence 1..69`: protocol valid but diagnostic-only
- `confidence 70..100`: usable for mapping when range is within mapping limits
- `confidence >100`: protocol invalid

`protocol_valid` means the payload fits the known device protocol. `usable_for_mapping` means it may later be considered by a mapper. M3-C3.1 only implements the contract; it does not add a new inverse sensor model.

## ToF FOV

The vendor described a spot range of about 260mm at 10m and 330mm at 12m. This stage treats that value as full spot width, not radius:

`full_fov = 2 * atan(spot_width / (2 * distance))`

Reference values:

- `10m, 0.260m -> 1.48960636 deg`
- `12m, 0.330m -> 1.57553465 deg`

Runtime default:

- `tof_full_fov_deg = 1.6`
- half angle is `0.8 deg`

The 1.6 degree value is a full cone angle. It is not a beam increment and must not generate multiple ranges.

## ToF Timestamp

ToF effective measurement time comes from the request window:

- `request_latency_s = response_received_s - request_start_s`
- `effective_timestamp_s = request_start_s + 0.5 * request_latency_s`

The clock source must be monotonic/steady. The echo tag, replay log write time, packet receive time, and outer loop time are not valid measurement time sources.

Three-ToF sync uses each frame's effective timestamp. With three sensors, synchronized time is the median. With `AllowMissingOne` and two sensors, it is the midpoint of the remaining two. One sensor is not a full multi-ToF snapshot mode.

## Snapshot And Legacy Projection

`RobotSlamSensorSnapshot.multi_tof` stores scalar front/left/right readings, including distance, confidence, echo tag, effective timestamp, protocol validity, mapping usability, FOV, frame id, and sync deltas.

The legacy `snapshot.tof` projection exists only for old backend compatibility. It contains at most one range and is marked with `legacy_scalar_tof_projection`. It does not mean the sensor is a 2D lidar.

## BL4820 Payload And Time

BL4820 read payload is 6 bytes:

- bytes 0-1: `position`, little-endian `uint16`
- bytes 2-3: `speed_rpm`, little-endian `int16`
- bytes 4-5: `current_raw`, little-endian `uint16`

Golden sample `E8 03 9C FF 0A 00` decodes to:

- `position = 1000`
- `speed_rpm = -100`
- `current_raw = 10`
- `current_A = 1.0`

Each successful wheel read uses:

- `timestamp_us = request_start_us + (response_received_us - request_start_us) / 2`
- `latency_us = response_received_us - request_start_us`

The left/right pair timestamp is the midpoint of the two successful wheel timestamps. Pair skew is their absolute difference. If skew exceeds the configured limit and odometry requires skew to be OK, the pair is rejected transactionally and no wheel state is partially committed.

## Freshness

Hold-last samples are diagnostic only:

- last good measurement timestamp is not rewritten
- reused values are not fresh
- EOF, timeout, checksum error, status error, pair skew failure, invalid replay, stale data, and snapshot build failure do not refresh app freshness

The stale gate must eventually trip if only reused values remain.

## Safety Boundary

M3-C3.1 remains fake/replay/offline only:

- `motion_execution_mode = dry_run`
- `hardware_write_enabled = false`
- `allow_writer_dispatch = false`
- `writer_backend = null`
- `production_interface_enabled = false`
- `translation_commands_enabled = false`
- `real_hardware_adapters_enabled = false`
- `pose_writeback_enabled = false`

The main runtime still uses null/fake writers. No `/dev` access is required for tests.

## Not Implemented

This stage does not implement real ToF UART, real BL4820 transport changes beyond timestamp semantics, real IMU integration, closed-loop 2D simulation, real sparse ToF SLAM backend, map quality for real SLAM, frontier exploration, A*, autonomous exploration, or real motion.

## Vendor TODO

Confirm whether the first 6 ToF bytes are only host echo, whether echo has units or auto-increments, whether 9 bytes is a complete frame or only payload, whether frame header/address/checksum exists, whether 20mm/12000mm endpoints are inclusive, exact confidence semantics, whether low confidence is "not recommended" or "must discard", whether 260mm/330mm are full width or radius, spot energy threshold, horizontal/vertical spot shape, 12m accuracy and confidence stability, material/strong light/angle effects, three-ToF polling/parallel mode, per-route latency, BL4820 position endpoint 1024, position/rpm direction, status bits, and 50-100Hz long-run stability.

## Next Stage

M3-C4 should connect the real-format scalar three-ToF replay into the fake autonomous SLAM pipeline. It should not claim true three-ToF map fusion.
