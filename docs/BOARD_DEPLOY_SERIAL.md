# RV1126B Serial Deploy Smoke

This note records the FT232 serial deployment path used for `robot_slamd` when
the board has no SSH/USB network link.

## Serial Link

Host side device:

```sh
/dev/ttyUSB0
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10QM814-if00-port0
```

The board console runs as `root` over the serial shell. If the host user cannot
open `/dev/ttyUSB0`, grant temporary access:

```sh
sudo chmod 666 /dev/ttyUSB0
```

Long-term access can be configured by adding the host user to `dialout` and
logging in again:

```sh
sudo usermod -aG dialout lab
```

## Local Binary Check

Build the ARM binary, then check type and md5 on the host:

```sh
file Projects/rv1126b-dev/robot_slamd/build/robot_slamd
md5sum Projects/rv1126b-dev/robot_slamd/build/robot_slamd
```

Known smoke binary from the 2026-06-29 board run:

```text
md5: 414736838352f2a4744b76e2662095a9
type: ELF 32-bit LSB pie executable, ARM, EABI5
```

Do not run the board binary if the board-side md5 does not match the host-side
md5.

## Create Board Directory

Confirm the shell and create the deploy directory:

```sh
Projects/rv1126b-dev/serial_cmd.py \
  "id; pwd; mkdir -p /userdata/robot_slamd/tools /userdata/robot_slamd_board_sim_spin_scan_smoke" \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --timeout 8
```

## Serial Upload

Use chunked serial upload with per-chunk md5 verification:

```sh
Projects/rv1126b-dev/serial_chunk_push.py \
  Projects/rv1126b-dev/robot_slamd/build/robot_slamd \
  /userdata/robot_slamd/robot_slamd \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --chunk-size 1024 \
  --line-chars 128 \
  --line-delay-ms 1 \
  --retries 5
```

Upload the board smoke config:

```sh
Projects/rv1126b-dev/serial_chunk_push.py \
  Projects/rv1126b-dev/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml \
  /userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --chunk-size 1024 \
  --line-chars 128 \
  --line-delay-ms 1 \
  --retries 5
```

The upload script prints both remote and local md5. They must match.

## Board Md5 Check

Verify the binary md5 on the board:

```sh
Projects/rv1126b-dev/serial_cmd.py \
  "md5sum /userdata/robot_slamd/robot_slamd; chmod +x /userdata/robot_slamd/robot_slamd" \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --timeout 8
```

Expected md5 for the 2026-06-29 smoke binary:

```text
414736838352f2a4744b76e2662095a9
```

## Board Smoke

Run the 20 second board smoke:

```sh
Projects/rv1126b-dev/serial_cmd.py \
  "/userdata/robot_slamd/robot_slamd --config /userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml --duration-s 20 --output-dir /userdata/robot_slamd_board_sim_spin_scan_smoke" \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --timeout 35
```

Check required outputs:

```sh
Projects/rv1126b-dev/serial_cmd.py \
  'RUN=/userdata/robot_slamd_board_sim_spin_scan_smoke/latest; ls -l "$RUN"; test -s "$RUN/config.resolved.yaml" && test -s "$RUN/map.pgm" && test -s "$RUN/map.yaml" && test -s "$RUN/trajectory.csv" && test -s "$RUN/localization_log.csv" && test -s "$RUN/tof_log.csv" && test -s "$RUN/metrics.csv" && test -s "$RUN/spin_scan.csv" && test -s "$RUN/spin_scan_bins.csv" && test -s "$RUN/spin_match_curve.csv" && test -s "$RUN/spin_match_summary.csv" && echo required_outputs_ok' \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --timeout 10
```

Minimum pass criteria:

```text
robot_slamd exits with code 0
latest directory exists
config.resolved.yaml exists
map.pgm and map.yaml exist
trajectory.csv exists
localization_log.csv exists
tof_log.csv exists
metrics.csv exists
spin_scan.csv exists when spin_scan_localization.enabled=true
spin_scan_bins.csv exists when spin_scan_localization.enabled=true
spin_match_curve.csv exists when spin_scan_localization.enabled=true
spin_match_summary.csv exists when spin_scan_localization.enabled=true
```

## Shell Fallback Runtime Profile

Some board images do not include `python3` or `file`. In that case, do not use
the Python board smoke wrapper on the board. Use the shell fallback tools:

```text
tools/board_runtime_profile_smoke.sh
tools/board_summarize_runtime_profile.sh
```

Upload them:

```sh
Projects/rv1126b-dev/serial_chunk_push.py \
  Projects/rv1126b-dev/robot_slamd/tools/board_runtime_profile_smoke.sh \
  /userdata/robot_slamd/tools/board_runtime_profile_smoke.sh \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --chunk-size 1024 \
  --line-chars 128 \
  --line-delay-ms 1 \
  --retries 5

Projects/rv1126b-dev/serial_chunk_push.py \
  Projects/rv1126b-dev/robot_slamd/tools/board_summarize_runtime_profile.sh \
  /userdata/robot_slamd/tools/board_summarize_runtime_profile.sh \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --chunk-size 1024 \
  --line-chars 128 \
  --line-delay-ms 1 \
  --retries 5
```

Run a profiled smoke:

```sh
Projects/rv1126b-dev/serial_cmd.py \
  "sh /userdata/robot_slamd/tools/board_runtime_profile_smoke.sh /userdata/robot_slamd/robot_slamd /userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml /userdata/robot_slamd_board_runtime_profile_smoke 20" \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --timeout 45
```

Summarize on the board:

```sh
Projects/rv1126b-dev/serial_cmd.py \
  "sh /userdata/robot_slamd/tools/board_summarize_runtime_profile.sh /userdata/robot_slamd_board_runtime_profile_smoke/latest" \
  --port /dev/ttyUSB0 \
  --baud 1500000 \
  --timeout 8
```

The shell fallback writes:

```text
/userdata/robot_slamd_board_runtime_profile_smoke/latest/runtime_profile.csv
```

Strict profile pass criteria:

```text
duration_s > 0
rss_mb_max < 200
cpu_percent_max < 200
```

Observed 2026-06-29 serial smoke result:

```text
duration_s,20.000
cpu_percent_mean,4.590
cpu_percent_max,7.805
rss_mb_mean,2.309
rss_mb_max,2.309
thread_count_max,1
strict,pass
```

## Safety Boundary

This deployment flow does not change C++ code, localization logic, mapping, yaw
correction, or pose writeback behavior. It only transfers files, runs board
smoke, and records runtime profile output.
