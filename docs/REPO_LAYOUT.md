# Repository Layout

## Current Problem

Before this cleanup, the repository root mixed production headers, test sources, configuration files, and design documents. R1 had already split some safety scaffolding internally, but the top-level layout still made review and ownership boundaries harder than necessary:

- Root-level production headers were too numerous.
- Root-level `test_*.cpp` files made test-only code easy to confuse with production code.
- Documentation and YAML configs were mixed with build entry points.
- R1 safety scaffolding was mechanically split, but not yet placed into stable project directories.

## Target Structure

```text
.
├── CMakeLists.txt
├── Dockerfile
├── README.md
├── README_CN.md
├── config/
├── docs/
├── include/
│   └── robot_slamd/
│       ├── app/
│       ├── active_scan/
│       ├── config/
│       ├── core/
│       ├── mapping/
│       ├── motion/
│       ├── recovery/
│       ├── sensors/
│       └── software_motion/
├── src/
├── tests/
└── tools/
```

`README.md` and `README_CN.md` are not present in this checkout; if they are added later, they should remain at the root.

## Files Moved In This Cleanup

- `src/`: main program entry point.
- `include/robot_slamd/app`: application orchestration header.
- `include/robot_slamd/core`: common types, grid, metrics, and metrics writer helpers.
- `include/robot_slamd/config`: config parser and motion-execution config helpers.
- `include/robot_slamd/sensors`: sensor and ToF headers.
- `include/robot_slamd/mapping`: localization, map quality, supervisor, and spin-scan localization headers.
- `include/robot_slamd/active_scan`: active scan, sparse scan, yaw matcher, and yaw correction lifecycle headers.
- `include/robot_slamd/recovery`: startup/lost recovery observe-only manager.
- `include/robot_slamd/motion`: motion safety executor, writer controller, preflight, and manual arm gate.
- `include/robot_slamd/software_motion`: software direction-speed command contract and shadow transports.
- `tests/`: all `test_*.cpp` files and the test-only fake transport header.
- `config/`: all YAML configuration files.
- `docs/`: design, checklist, deployment, and refactor audit documents.

## Behavior Not Changed

This cleanup is layout-only. It does not change:

- MotionSafetyExecutor safety gate semantics.
- MotionWriteController writer fault latch semantics.
- fail-closed config defaults or fatal validation strings.
- metrics field names or output order.
- ctest test names.
- Localizer, mapping, ToF, sparse scan, yaw correction, recovery, or BL4820 read behavior.
- The absence of real motor writes or real software transport.

## Regression Strategy

The migration is protected by:

- Project-relative includes under `robot_slamd/...`.
- CMake test helper updated to compile from `tests/` with `include` and `tests` include paths.
- `test_refactor_include_boundaries` updated to include the new public paths.
- Full `cmake --build`, `ctest`, sim smoke, hardware fatal smoke, and static danger-path scans.

## Deferred To R2

R2 may consider deeper structural changes, but those should remain separate from this mechanical layout move:

- Split `Config` into grouped structures.
- Split `RunMetrics` into grouped writers with stronger type boundaries.
- Move Docker/compose assets into a dedicated docker layout if deployment commands are updated together.
- Extract shared test fixtures and helpers.
- Introduce stricter include visibility rules if the project grows beyond header-only modules.

## Algorithm Motion API

M2-B2 adds `include/robot_slamd/motion/algorithm_motion_command*.hpp` and `include/robot_slamd/motion/algorithm_motion_facade.hpp`. The user-facing contract is documented in `docs/ALGORITHM_MOTION_API.md`. These files define high-level algorithm intent only; they do not add a real transport or hardware write path.

## M2-B3 Software Transport Spec Files

Software-side transport handoff documents live in `docs/SOFTWARE_TRANSPORT_IMPLEMENTATION_SPEC.md`, `docs/SOFTWARE_TRANSPORT_ACCEPTANCE_TESTS.md`, and `docs/SOFTWARE_TRANSPORT_GOLDEN_COMMANDS.md`. Shadow-only contract helpers live under `include/robot_slamd/software_motion/`.

## M3-A0 Autonomy Layout

M3-A0 adds `include/robot_slamd/autonomy/` for the hardware-ready autonomous SLAM coordinator, policy, types, and ports. Tests live under `tests/test_autonomous_slam_*` and `tests/test_robot_slam_ports.cpp`. The module has no real robot driver or runtime transport dependency; real robot integration should add adapters without changing the coordinator. See `docs/HARDWARE_READY_AUTONOMOUS_SLAM_CORE.md`.

## M3-A1 Adapter Contract Layout

Real adapter contracts live under `include/robot_slamd/autonomy/contracts/`. Replay-only adapters live under `include/robot_slamd/autonomy/adapters/`. They are contract and test scaffolding only; real driver implementations are intentionally not present in M3-A1.

M3-A2 adds include/robot_slamd/autonomy/prelive/ for the pre-live autonomous SLAM runner, gate, types, and text report writer.

M3-A3 adds include/robot_slamd/autonomy/map_backend/ for SLAM backend binding contracts, adapter skeleton, replay binding, and acceptance runner.
M3-A4 adds `docs/AUTONOMOUS_SLAM_E2E_PRELIVE_SCENARIO.md` for the end-to-end pre-live shadow scenario across replay sensors, SLAM backend binding, map port, pre-live runner, coordinator, policy, and shadow motion.

## M3-B0 Autonomy Stub Layout

- `include/robot_slamd/autonomy/real_adapters/`: fail-closed real adapter stubs and factory.
- `include/robot_slamd/autonomy/live_handoff/`: live handoff readiness checker.
- `docs/REAL_ADAPTER_STUBS.md`: M3-B0 stub overview.
- `docs/LIVE_HANDOFF_READINESS_CHECKLIST.md`: gates before lifted live preparation.
- `docs/REAL_ADAPTER_IMPLEMENTATION_TODO.md`: owner TODO list for future real adapters.

## M3-B1 Sensor Data Contract Layout

- `include/robot_slamd/autonomy/real_adapters/sensor_data/`: raw sensor packet contract, checker, builder, acceptance runner, report writer, and sample data.
- `docs/REAL_SENSOR_ADAPTER_DATA_CONTRACT.md`: request-based ToF/Wheel timing and raw packet rules.
- `docs/REAL_SENSOR_ADAPTER_ACCEPTANCE_TESTS.md`: M3-B1 acceptance checklist.
- `docs/REAL_SENSOR_FIELD_MAPPING_TODO.md`: future real sensor field mapping TODOs.

## M3-B2 Sensor Replay Layout

`include/robot_slamd/autonomy/real_adapters/sensor_replay/` contains offline replay/log contract headers for request-window sensor data regression.

## M3-B2.1 Replay Robustness Files

- `include/robot_slamd/autonomy/real_adapters/sensor_replay/`: strict offline real sensor replay codec, port, acceptance, and report utilities.
- `include/robot_slamd/autonomy/real_adapters/sensor_replay/regression/`: golden replay regression harness skeleton.
- `docs/REAL_SENSOR_REPLAY_ROBUSTNESS.md`: replay robustness notes.
- `docs/REAL_SENSOR_REPLAY_GOLDEN_REGRESSION.md`: golden regression harness notes.

## M3-B3 Deterministic SLAM Backend Files

- `include/robot_slamd/autonomy/real_adapters/slam_backend/`: deterministic SLAM backend skeleton, ToF evaluator, map quality tracker, binding, and report writer.
- `include/robot_slamd/autonomy/real_adapters/slam_backend/regression/`: offline replay-to-backend regression runner and report writer.
- `docs/DETERMINISTIC_SLAM_BACKEND_SKELETON.md`: backend skeleton notes.
- `docs/REPLAY_TO_SLAM_BACKEND_REGRESSION.md`: replay-to-map regression notes.
- `docs/REAL_SLAM_BACKEND_IMPLEMENTATION_TODO.md`: future production backend TODO.

- `include/robot_slamd/autonomy/full_pipeline/`: M3-B4 full autonomous SLAM fake/replay pipeline runner, scenario builder, fake motion port, and report writer.
