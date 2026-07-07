# R1 Refactor Audit

This audit covers the safe mechanical refactor after M2-B1-pre2 for Xiaoen `robot_slamd` / `en`.

## Current File Responsibilities

- `config.hpp`: Parses YAML-like config, validates all config groups, and writes resolved config. It remains the public config entry point.
- `common.hpp`: Holds shared data structures, runtime config fields, snapshots, stats, and small utility functions.
- `metrics.hpp`: Writes `metrics.csv` for all subsystems.
- `CMakeLists.txt`: Defines the main `robot_slamd` executable and all unit tests.
- `software_motion_command.hpp`: Defines the direction/speed software motion command contract and validation.
- `software_motion_transport.hpp`: Defines the abstract software motion transport interface.
- `software_direction_speed_motion_command_writer.hpp`: Adapts wheel-rpm intents into software direction/speed commands through a transport abstraction.
- `motion_hardware_preflight.hpp`: Implements M2-B1 lifted preflight checks, including direction probe versus confirmed-live modes.
- `manual_motion_arm_gate.hpp`: Implements the manual arming phrase and checkbox gate for future live tests.
- `motion_command_writer.hpp`: Defines the generic motion writer interface plus null/fake RPM writer scaffolding.
- `motion_write_controller.hpp`: Dispatches motion safety snapshots to a writer only when explicitly allowed, with writer-fault latching.
- `bl4820_motion_safety_executor.hpp`: Computes dry-run wheel targets and safety-gate results. It does not write motors.
- `app.hpp`: Wires snapshots, logging, metrics, and dry-run motion safety execution together.
- `main.cpp`: Process entry point only; it delegates to `real_main`.
- `test_*.cpp`: Unit and static-style regression tests for mapping, yaw correction, recovery, motion safety, software motion, and M2-B1 preflight scaffolding.

## Coupling Points

- `config.hpp` was coupled to motion-execution parsing and validation through a long shared function.
- `metrics.hpp` was coupled to software-motion metrics through a long contiguous write block.
- `software_motion_transport.hpp` mixed the production transport interface with test fake and loopback shadow implementations.
- `CMakeLists.txt` repeated identical test target registration for every test.

## Mechanical Splits In R1

- Added `config_motion_execution.hpp` for motion-execution parse/validate helpers. Resolved-config output intentionally stays in `config.hpp` to preserve key order.
- Added `software_motion_metrics_writer.hpp` and moved the software-motion/M2-B1 metrics write block into `write_software_motion_metrics` with field names and order preserved.
- Split transport headers:
  - `software_motion_transport.hpp` keeps only `SoftwareMotionSendResult` and `SoftwareMotionCommandTransport`.
  - `test_software_motion_transport_fakes.hpp` holds `FakeSoftwareMotionCommandTransport` and is test-only.
  - `loopback_software_motion_transport.hpp` holds `LoopbackSoftwareMotionCommandTransport`, which remains shadow-only and does not touch hardware.
- Added `add_robot_slamd_test()` in `CMakeLists.txt` and preserved all test names.
- Added `test_refactor_include_boundaries.cpp` for include and fake-boundary regression.

## Explicitly Not Changed

- No Localizer, map update, ToF filter, yaw matcher, yaw correction, recovery, or motion safety algorithms were rewritten.
- No resolved-config key names were changed.
- No metrics field names or order were changed.
- No config defaults were changed.
- No safety-gate error strings were changed.
- No real transport was added.
- No BL4820 write path was added.

## Why Safety Behavior Is Unchanged

The refactor only moves declarations or helper blocks and updates include sites. The motion write controller still requires explicit dispatch enablement, and production config still fails closed for writer dispatch, hardware write, non-shadow loopback, and production interface enablement. Fake transport is no longer present in the production transport header. Loopback transport still only records commands and rejects non-shadow mode when required.

## R2 Candidates

- Split the full `Config` structure into grouped sub-structures only after adding stronger resolved-config golden tests.
- Split `RunMetrics` into grouped writer modules for all subsystems, not only software motion.
- Extract common test helpers after preserving current test behavior with golden outputs.
- Consider moving resolved-config motion execution writing only with a field-order regression test.
