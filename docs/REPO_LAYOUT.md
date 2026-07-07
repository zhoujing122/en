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
