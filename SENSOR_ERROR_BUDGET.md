# Sensor Error Budget

This document defines the first-version engineering error budget for `robot_slamd`.
The values below are acceptance ranges for calibration and field validation, not
manufacturer datasheet limits. If a sensor exceeds these ranges, keep the run in
diagnostic mode and do not treat the map as a calibration-quality result.

## Summary

| Item | Required range after calibration | Hard reject / warning condition | Where to check |
| --- | --- | --- | --- |
| `encoder_ticks_per_rev` | Exact configured count must match the encoder/gearbox output count. Measured 10 wheel turns should be within 1 tick/turn of the configured value. | Any unknown gearbox ratio, quadrature multiplier, or left/right count mismatch over 0.5%. | `config.*.yaml`, encoder raw CSV |
| Left/right wheel radius | Each side within 1% of the distance-derived value. Straight 2 m run distance error <= 2%. | Left/right radius mismatch causing straight 2 m lateral drift > 5 cm. | `trajectory.csv`, tape-measured straight run |
| `wheel_base_m` | In-place 360 deg turn yaw error <= 5 deg after calibration; target <= 3 deg. | In-place turn error > 8 deg or repeated spin runs disagree by > 3 deg. | `localization_log.csv`, marked floor yaw |
| Encoder sample timing | Timestamp monotonic; jitter <= 10 ms for 50 Hz localization input. | Non-monotonic timestamps, gaps > 100 ms, or frequent `encoder_gap` / `encoder_jump`. | `localization_log.csv` |
| IMU gyro z bias | Static bias magnitude <= 0.03 rad/s after startup calibration; target <= 0.01 rad/s. | `imu_calib_failed`, repeated `imu_gap`, or `gyro_spike`. | `localization_log.csv`, `metrics.csv` |
| IMU yaw direction | Positive robot counterclockwise spin produces positive mapped gyro z and positive yaw. | Sign/axis mismatch in a manual spin test. | `imu.gyro_axis`, `imu.gyro_sign`, `localization_log.csv` |
| 3-ToF range, 0.02-0.5 m | Absolute error <= 30 mm after offset/crosstalk calibration. | Systematic error > 50 mm at a fixed wall. | `tof_log.csv`, tape-measured wall target |
| 3-ToF range, 0.5-5.0 m | Absolute error <= 50 mm or <= 3%, whichever is larger. | Error > 80 mm or unstable confidence in normal indoor lighting. | `tof_log.csv` |
| 3-ToF invalid value | `confidence=0` means invalid/no measurement and must not update occupied or free cells. | Driver maps invalid confidence to `RangeStatus=255` or any other free-space update. | `tof_log.csv`, `metrics.csv` |
| 3-ToF confidence | Valid mapping hits should normally have confidence >= 70. | Per-route `confidence_mean < confidence_min` or unhealthy route. | `tof_log.csv`, `metrics.csv` |
| ToF mounting yaw | Front <= 2 deg from forward; left/right <= 2 deg from +/-90 deg after calibration. | Wall runs show consistent yaw residual > 3 deg. | `tof_extrinsics.*.yaw_deg`, `candidate_yaw.csv` |
| ToF mounting x/y | Sensor origin measured within 10 mm of configured `x_m/y_m`. | Unknown or unmeasured mounting offset. | `tof_extrinsics.*.x_m/y_m` |
| Cross-sensor timestamp skew | Front/left/right samples within 20 ms for mapping. | Skew > 50 ms during motion. | `tof_log.csv` |

## Encoder And Wheel Calibration

Use cumulative ticks at the `Localizer` boundary. For CJC BL4820 UART motors,
`robot_slamd` reads single-turn 0..1023 position counts and unwraps them into
cumulative ticks internally.

Minimum calibration runs:

```text
1. Wheel suspended, rotate each wheel 10 turns:
   configured encoder_ticks_per_rev must match measured_delta / 10.

2. Straight run, 2-5 m:
   left/right wheel radius is derived from measured distance and tick delta.

3. In-place turn, 3 full rotations:
   wheel_base_m is derived from left/right distance difference and yaw delta.
```

Acceptance:

```text
straight_2m_distance_error <= 0.04 m
straight_2m_lateral_drift <= 0.05 m
turn_360_yaw_error <= 5 deg
encoder_jump count == 0 in the calibration run
```

## IMU Gyro

The first version uses gyro z only for yaw correction. It does not use
accelerometer integration, magnetometer heading, quaternion attitude, or direct
IMU yaw.

Acceptance:

```text
static gyro_z bias target: <= 0.01 rad/s
static gyro_z bias maximum: <= 0.03 rad/s
gyro spike threshold: 4.0 rad/s
manual CCW spin: mapped gyro_z must be positive
manual CW spin: mapped gyro_z must be negative
```

If startup calibration fails, keep the run for diagnostics only unless the IMU is
intentionally disabled or in `log_only` mode.

## 3-ToF

Validate each route independently: `front`, `left`, and `right`.

Use flat wall targets at known distances:

```text
0.03 m, 0.10 m, 0.30 m, 0.50 m, 1.00 m, 2.00 m, 5.00 m
```

Acceptance:

```text
0.02-0.50 m: absolute error <= 0.03 m
0.50-5.00 m: absolute error <= max(0.05 m, measured_range * 0.03)
frame rate: 30 Hz per route
distance unit: mm
confidence >= 70 for mapping hits
confidence == 0 for invalid/no-measurement samples
no-status ToF drivers synthesize RangeStatus 0 and let confidence gate validity
RangeStatus 255 is allowed only when a device truly reports no-object/free-only
RangeStatus 6 is log-only and must not mark occupied cells when provided
front/left/right optical spots must not overlap on the same target in normal mounting
```

Reject a route for mapping until fixed if:

```text
gap ratio > 50% after at least 20 samples
reject rate > 80% after at least 20 samples
confidence_mean < confidence_min after at least 20 confidence samples
repeated tof_read_fail / tof_missing_<id>
```

## Extrinsics

The default ToF placement values are only starting values. Measure final mounted
sensor origins from `base_link`, whose origin is the drive wheel axle center.

Acceptance:

```text
x_m/y_m measurement error <= 0.01 m
front yaw error <= 2 deg
left yaw error <= 2 deg from +90 deg
right yaw error <= 2 deg from -90 deg
```

If `yaw_candidate` repeatedly reports low margin, flat curve, multimodal result,
or best-at-search-edge, do not use the run to approve extrinsics.

## Timestamp Quality

Timestamp quality affects both odometry integration and ToF projection.

Acceptance:

```text
encoder timestamps monotonic
imu timestamps monotonic when using CSV
tof timestamps monotonic per sensor
encoder localization input jitter <= 10 ms
tof route skew <= 20 ms target, <= 50 ms maximum while moving
request-mode ToF timestamp_us must be captured before request send and echoed in response
```

Any non-monotonic timestamp stream must be fixed at the data source.
