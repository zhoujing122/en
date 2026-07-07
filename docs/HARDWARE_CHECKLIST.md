# robot_slamd Hardware Bring-up Checklist

Before accepting any hardware run, verify the sensor error budget in `docs/SENSOR_ERROR_BUDGET.md`.

## Board and transfer

- Board serial console is visible on the host, normally `/dev/ttyUSB0`.
- Serial baud rate is `1500000`.
- Prefer network `scp/rsync` if the board has network. If serial-only, use `serial_chunk_push.py`.
- Do not run the binary unless local and remote MD5 match.
- Runtime directory on board: `/userdata/robot_slamd`.
- Smoke output directory on board: `/userdata/robot_slamd_algo_opt`.

## Required smoke outputs

After a board smoke run, check these files under `latest/`:

- `config.resolved.yaml`
- `map.pgm`
- `map.yaml`
- `trajectory.csv`
- `localization_log.csv`
- `tof_log.csv`
- `metrics.csv`

## 3-ToF Wiring

Record the final mapping before running real ToF:

| Logical route | Physical position | Linux device | Notes |
| --- | --- | --- | --- |
| front | front-facing | `/dev/mt3801_front` | Confirm by blocking front sensor only. |
| left | robot left | `/dev/mt3801_left` | Confirm by blocking left sensor only. |
| right | robot right | `/dev/mt3801_right` | Confirm by blocking right sensor only. |

Checks:

- Power rail matches the ToF module requirement.
- Board GND and sensor GND are common.
- Each route can be read independently.
- Blocking one sensor changes only that route in `tof_log.csv`.
- Distance is logged in mm and converted correctly in `tof_log.csv`.
- Request-mode `timestamp_us` is monotonic and echoed back by the ToF response.
- `confidence=0` appears for invalid/no-measurement samples and does not update the map.
- `0 < confidence < 70` is logged as low confidence and rejected for mapping.
- If the ToF has no status bit, the driver must synthesize `range_status=0`; do not synthesize `255` from `confidence=0`.
- Front/left/right optical spots should not overlap on the same target during normal mounting.
- Watch for `tof_read_fail`, `tof_gap`, `tof_missing_front`, `tof_missing_left`, `tof_missing_right`.

## Encoder wiring

- Left/right encoder channels are not swapped.
- Forward push increases both tick counts.
- In-place left turn and right turn produce opposite left/right deltas.
- Wheel radius and wheel base are measured, not left at defaults for real mapping.
- Watch for `encoder_gap` and `encoder_jump` in `localization_log.csv`.

## ICM43600 IMU wiring and mounting

- Check `/sys/bus/iio/devices/iio:device*/name` and `/dev/iio:device*` first.
- Current target only uses angular velocity around robot yaw.
- If the IMU is mounted on the chassis, set `imu.gyro_axis` and `imu.gyro_sign` to map the physical axis to robot yaw.
- Start with `imu.mode: log_only` until static gyro and turn direction are confirmed.
- Switch to `imu.mode: localization` only after the sign and axis are correct.
- Watch for `imu_gap`, `imu_read_fail`, `gyro_spike`, and `final_gyro_bias_rad_s`.

## First real-data run order

1. Run board sim smoke.
2. Run ToF-only read smoke and inspect `tof_log.csv`.
3. Run encoder-only motion smoke and inspect `trajectory.csv` plus warning flags.
4. Run IMU in `log_only` and confirm axis/sign.
5. Enable IMU localization.
6. Push slowly for a short mapping run.

## First acceptance criteria

- Program runs for 20 seconds without crash.
- All required output files exist and are non-empty.
- `config.resolved.yaml` contains the effective final config.
- ToF confidence appears in `tof_log.csv`.
- Bad odom pauses mapping instead of polluting the map.
- No unexpected silent fallback to sim when a real/csv source is configured.
