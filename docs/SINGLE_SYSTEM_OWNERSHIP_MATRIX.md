# Single Production System Ownership Matrix

## Scope and baseline

This inventory is the deletion gate for consolidating `robot_slamd` from
`49cb33eaa755c56da6576da17580d1383a507761`.  It describes production
ownership, not the number of independently constructible test fixtures.  No
legacy or shadow implementation may be removed until its replacement owner is
wired through the executable and the listed parity tests pass.

The required production call chain is:

```text
Simulation / Replay / Real sensor adapter
  -> RobotSlamSensorSnapshot
  -> RobotSlamApplication::step(...)
  -> SparseSlamRuntimeCore::step(...)
  -> AutonomousExplorationController (when operation=exploration)
  -> SafeWaypointTracker
  -> RobotSlamMotionPort safety boundary
  -> SoftwareMotionCommandTransport
```

`SensorSource` selects only an I/O adapter. `OperationMode` selects policy
(`mapping`, `localization`, or `exploration`). Neither selector may instantiate
another odometry, map, matcher, correction, recovery, or SLAM pipeline.

## Unique production owners

| Capability | Current production-capable implementations | Unique production owner after consolidation | Boundary / invariant |
|---|---|---|---|
| Sensor reading and snapshot publication | Legacy `SensorManager`; `SimSensorPort`; `MultiTofReplaySensorPort` / replay codecs; fail-closed real adapter stubs; SparseShadow deterministic sensor | A `RobotSlamSensorPort` adapter chosen by `SensorSource`, returning `RobotSlamSensorSnapshot` | Adapters perform I/O and contract conversion only; they never own SLAM state. |
| Three scalar ToF Hit / NoReturn / Invalid semantics | Multi-ToF contract, snapshot builder, simulation sensors, sparse observation builder | `ScalarTofSnapshotFrame` plus `SparseTofObservationBuilder` | The canonical snapshot preserves protocol validity and mapping usability; ground truth is never attached. |
| Wheel/IMU dead reckoning | Legacy `Localizer`; canonical `WheelImuDeadReckoning2D` in Core | `SparseSlamRuntimeCore::estimator_` (`WheelImuDeadReckoning2D`) | Commands are never odometry input; only wheel feedback and IMU measurements are accepted. |
| Measurement-time odom pose | Legacy latest-pose update; canonical timed interpolation | `SparseSlamRuntimeCore::pose_buffer_` (`TimedOdomPoseBuffer`) and `MultiTofMeasurementPoseResolver` | Each ToF ray uses its effective measurement time. |
| map/odom frame and `map_T_odom` | Legacy pose mutation / correction path; canonical `MapOdomFrameState` | `SparseSlamRuntimeCore::frame_state_` | The Core is the sole writer. Startup, local match, and recovery changes are transactional. |
| Phase-aware observation bundle | Legacy active scan collector; canonical phase-aware controller | `SparseSlamRuntimeCore::observation_controller_` (`PhaseAwareObservationController`) | Mapping is frozen while a bundle is active; reference revision is guarded. |
| Matcher scoring | Legacy `TofPoseCorrector`, `SpinScanLocalizer`, and `SparseScanYawMatcher`; canonical sparse matcher and relocalizer share sparse ray scoring | `SparseTofLocalMatcher` / `SparseMultiTofRelocalizer` using the scoring primitives in `localization/sparse_tof` | Yaw-only local matching remains the production local mode; bounded SE(2) is used only for relocalization. There is no second production scoring implementation. |
| Writable occupancy map | Legacy `ChunkedGrid`; canonical sparse backend grid | `SparseMultiTofOccupancyBackendBinding` owned only by `SparseSlamRuntimeCore` | Adapters and controllers receive immutable snapshots; only the Core commits writes. |
| Keyframe and map commit | Legacy independent map updates/corrections; canonical atomic preparation/commit | `SparseSlamRuntimeCore::execute_prepared_local_match` coordinating `SparseTofKeyframeManager` and `SparseTofKeyframeMapTransaction` | correction, keyframe, map cells, bundle consumption, and revision advance form one transaction. |
| Map save/load | Legacy PGM/YAML `ChunkedGrid`; canonical `.smap` artifact lifecycle | `SparseSlamRuntimeCore::{initialize,save_sparse_map}` and `sparse_map_artifact.hpp` | Compatibility is checked before loaded cells are installed. |
| ConfiguredPose startup | Separate Legacy startup behavior and canonical initialization contract | `SparseSlamRuntimeCore::initialize` plus `MapOdomFrameState` | `map_T_odom` is initialized once from the configured map pose. |
| Startup relocalization | Legacy `RecoveryManager`; canonical manager/relocalizer | `SparseSlamRuntimeCore` coordinating `SparseRelocalizationManager` and `SparseMultiTofRelocalizer` | Requires two independent bundles; map cells, revision, and keyframes remain unchanged. |
| Localization health | Legacy quality/supervisor heuristics; canonical health tracker | `SparseSlamRuntimeCore::localization_health_` (`LocalizationHealthTracker`) | Ordinary ambiguity may degrade health but does not by itself declare Lost. |
| Lost recovery | Legacy `RecoveryManager`; canonical local/global recovery | `SparseSlamRuntimeCore` coordinating `LocalizationHealthTracker`, `SparseRelocalizationManager`, and `SparseMultiTofRelocalizer` | Lost stops motion immediately, blocks map/keyframes, escalates local to global, then atomically replaces `map_T_odom`. |
| Frontier and A* | Canonical M3-E exploration modules | `AutonomousExplorationController` using `SparseFrontierPlanner`, `NavigationGridView`, and `BoundedAStar` | Planner consumes immutable Core map snapshots and current map pose. |
| Waypoint tracking | `SafeWaypointTracker` | `AutonomousExplorationController::tracker_` (`SafeWaypointTracker`) | A stale path is cleared after relocalization/recovery and replanned. |
| Motion safety/controller | Legacy `BL4820MotionSafetyExecutor`; `SafeWaypointTracker`; algorithm motion facade and transport contract | `RobotSlamMotionPort` production safety boundary, with `SafeWaypointTracker` for navigation commands and retained BL4820/manual-arm/preflight checks at the real adapter boundary | Emergency stop and disabled-write policy take precedence over all algorithm commands. |
| Motor transport | Null/loopback/shadow software transports and hardware writer paths | One injected `SoftwareMotionCommandTransport`, reached through a `RobotSlamMotionPort` adapter | Real writes are disabled by default; unavailable or unarmed hardware fails closed. |
| Application composition | Legacy `real_main`, `SparseShadowRuntime`, and `M3EExplorationRunner` each compose a runtime | `RobotSlamApplication` | The executable constructs exactly one Application, which constructs exactly one Core. |

## Source classification

### KEEP

- Canonical types and ports under `autonomy/autonomous_slam_types.hpp` and
  `autonomy/ports/`.
- UART/BL4820 protocol, scalar ToF codec and contract, IMU/encoder input, replay
  codecs, and multi-ToF snapshot builders.
- `runtime/sparse_slam_runtime_core.hpp`, sparse mapping/localization/frame,
  relocalization, health, and map artifact modules.
- `exploration/` Frontier/A*/tracker/controller modules.
- Software motion transport, algorithm command facade, manual arm/preflight,
  and BL4820 safety modules after their legacy recovery-type dependency is
  removed.
- Simulation clock/world/plant/sensors/motion port. The world/plant pose stays
  inside simulation and acceptance reporting; it is not algorithm input.
- Tests that exercise the unique Core, contracts, hardware protocols, safety,
  replay conversion, simulation isolation, and behavioral acceptance.

### MIGRATE

- `M3EExplorationRunner`: move Core/controller ownership and per-step dispatch
  into `RobotSlamApplication`; retain the bounded simulation driver and report
  conversion as an adapter/runner around the Application.
- SparseShadow deterministic sensor generation: move to a simulation test
  adapter; replace the shadow report copy with a unified application/Core
  report view.
- Replay ports: expose their existing `RobotSlamSensorSnapshot` through the
  same Application entry used by simulation.
- Real adapter factory/stubs and hardware protocol readers: present the
  canonical sensor and motion ports. Until live composition is complete,
  `sensor_source=real` must reject startup explicitly.
- BL4820 motion safety: retain hardware checks while replacing dependencies on
  the old `RecoveryManager` snapshot with canonical Application/Core health.

### REMOVE after entry switch and parity tests

- `runtime/sparse_shadow_runtime.hpp` and the `SparseShadow` runtime mode/report.
- The Legacy branch of `app/app.hpp` and its direct construction of
  `Localizer`, `ChunkedGrid`, `TofPoseCorrector`, `SpinScanLocalizer`,
  `SparseScanCollector`, `SparseScanYawMatcher`, and old `RecoveryManager`.
- Second-pipeline headers used only by that entry:
  `mapping/localizer.hpp`, `core/grid.hpp`,
  `sensors/tof_pose_correction.hpp`,
  `mapping/spin_scan_localization.hpp`,
  `active_scan/sparse_scan_collector.hpp`,
  `active_scan/sparse_scan_yaw_matcher.hpp`, and
  `recovery/startup_lost_recovery_manager.hpp`, together with legacy-only
  correction/quality/supervisor glue once no retained hardware safety module
  depends on it.
- Tests/configuration whose only purpose is to prove a separately selectable
  Legacy or SparseShadow runtime. Capability tests are retained or replaced
  before those tests are removed.
- Fake/deterministic pre-Core product pipelines that create a second SLAM/map
  backend are test-history only and must not be linked into `robot_slamd`.

### ARCHIVE-ONLY

- Historical reports, stage inventories, and WIP design notes remain as source
  history/documentation but are not production includes or CMake runtime
  sources.
- `archive/main-wip-20260716` is read-only reference material. Its observability
  scenarios and motor/encoder documentation may inform tests/docs; its pose,
  bundle, matcher, keyframe, grid, and local SLAM pipeline are not copied.

## Feature parity and acceptance mapping

| Retained behavior | Primary acceptance tests after consolidation |
|---|---|
| Scalar ToF Hit/NoReturn/Invalid | `test_scalar_tof_return_semantics`, `test_sparse_tof_observation_builder` |
| Wheel/IMU odometry from feedback | `test_wheel_imu_dead_reckoning_2d` |
| Measurement-time pose | `test_timed_odom_pose_buffer`, `test_sparse_slam_runtime_core_timed_pose`, `test_sparse_backend_measurement_time_poses` |
| Phase-aware bundle and map freeze | `test_phase_aware_observation_controller`, `test_sparse_slam_runtime_core_active_bundle`, `test_sparse_slam_runtime_core_map_commit_gate` |
| Yaw-only local match and ambiguity rejection | `test_sparse_tof_yaw_match_unique`, `test_sparse_tof_yaw_match_rejections`, `test_sparse_slam_runtime_core_yaw_match_execution` |
| Atomic correction/keyframe/map commit | `test_sparse_slam_runtime_core_atomic_commit`, `test_sparse_slam_runtime_core_atomic_rollback`, `test_sparse_slam_runtime_core_atomic_determinism` |
| Sparse map save/load | `test_sparse_map_artifact_roundtrip`, `test_sparse_slam_runtime_core_map_lifecycle` |
| ConfiguredPose and startup relocalization | `test_sparse_slam_runtime_core_initialization`, `test_sparse_slam_runtime_startup_relocalization`, `test_m3d3b_startup_relocalization_simulation` |
| Symmetric relocalization rejection | `test_sparse_multi_tof_relocalizer`, `test_sparse_relocalization_manager`, consolidated Application rejection scenario |
| Localization health and lost recovery | `test_localization_health_tracker`, `test_sparse_slam_runtime_lost_recovery`, `test_m3d3b_lost_recovery_simulation` |
| Recovery failure remains stopped | `test_sparse_slam_runtime_lost_recovery`, consolidated Application failure scenario |
| Frontier/A*/tracker | `test_sparse_frontier_astar`, `test_frontier_goal_quality`, `test_safe_waypoint_tracker` |
| Deterministic create-new-map exploration | `test_m3e_autonomous_exploration`, `test_m3e_exploration_determinism` |
| Ground-truth isolation | `test_m3e_exploration_oracle_boundary`, `test_m3d1_ground_truth_isolation` |
| Replay snapshot -> Application -> Core | consolidated `test_robot_slam_application_replay` plus retained replay codec/port tests |
| Real adapter unavailable fails closed | consolidated Application configuration/composition tests plus retained real adapter stub tests |
| Motion writes disabled by default | consolidated Application composition test plus `test_manual_motion_arm_gate`, `test_motion_hardware_preflight`, and software transport contract tests |

## Structural deletion gates

Before the final removal commit, static/structural tests must establish:

1. `src/main.cpp` reaches one composition function that constructs one
   `RobotSlamApplication`.
2. `RobotSlamApplication` contains one `SparseSlamRuntimeCore` and no second
   writable map or odometry estimator.
3. The executable include graph contains no `Localizer`, `ChunkedGrid`, legacy
   recovery, or SparseShadow runtime.
4. Simulation and replay adapters call the same Application/Core types.
5. Real selection and unknown source/operation values fail closed.
6. Deprecated `runtime.slam_runtime_mode` and `slam_runtime.mode` keys are
   rejected rather than translated.
7. CMake does not compile or register Legacy/Shadow runtime-path tests as
   production parity evidence.
8. Repository static ownership checks find one production matcher-scoring
   owner, one `map_T_odom` owner, and one writable sparse occupancy owner.
