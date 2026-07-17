# Single-system feature parity

This is the feature-level acceptance record for the production path after the
Legacy and SparseShadow runtime selectors were retired. A fixture may construct
an isolated contract object for a unit test, but no fixture is a second
production SLAM pipeline.

| Feature | Unique owner | Tests / evidence | Retained | Hardware TODO |
|---|---|---|---|---|
| Three scalar ToF routes | `ScalarTofSnapshotFrame`, `SparseTofObservationBuilder` | `test_scalar_tof_9byte_codec`, `test_sparse_tof_observation_builder`, formal Simulation reports | yes | real ToF adapter |
| Hit / NoReturn / Invalid | scalar ToF contract and observation builder | `test_scalar_tof_return_semantics`, `test_multi_tof_snapshot_builder_failure_modes` | yes | real sensor validation |
| Request/effective timestamps | replay/snapshot builder and measurement pose resolver | `test_multi_tof_effective_time_policy`, `test_sparse_backend_measurement_time_poses` | yes | timestamp calibration |
| Wheel/IMU odometry | `SparseSlamRuntimeCore::estimator_` | `test_wheel_imu_dead_reckoning_2d`, structure audit | yes | encoder/IMU hardware adapter |
| Measurement-time pose | `TimedOdomPoseBuffer` and resolver in Core | `test_timed_odom_pose_buffer`, `test_sparse_slam_runtime_core_timed_pose` | yes | sensor clock calibration |
| map/odom frame | `MapOdomFrameState` in Core | `test_map_odom_frame_state`, Core report | yes | real frame calibration |
| Phase-aware bundle | Core observation controller | `test_phase_aware_observation_controller`, `test_sparse_slam_runtime_core_active_bundle` | yes | none |
| Immutable reference snapshot | Core reference-map preparation | `test_sparse_reference_map_snapshot*`, `test_sparse_slam_runtime_core_reference_snapshot` | yes | none |
| Yaw-only local matcher | `SparseTofLocalMatcher` in Core | `test_sparse_tof_yaw_match_*`, `test_sparse_slam_runtime_core_yaw_match_execution` | yes | real sensor tuning |
| Bounded candidate scoring | sparse matcher scoring owner | matcher rejection/determinism tests | yes | production parameters |
| Ambiguity/rejection gates | local matcher and relocalizer | `test_sparse_tof_yaw_match_rejections`, `test_sparse_multi_tof_relocalizer` | yes | field thresholds |
| Atomic correction/keyframe/map transaction | Core + keyframe transaction | `test_sparse_slam_runtime_core_atomic_*`, `test_sparse_keyframe_map_transaction*` | yes | none |
| Writable sparse occupancy map | Core sparse backend | `test_sparse_occupancy_grid`, `test_sparse_multi_tof_backend`, formal map revision/cells | yes | map scale tuning |
| Map save/load | Core `.smap` lifecycle | `test_sparse_map_artifact_roundtrip`, `test_sparse_slam_runtime_core_map_lifecycle` | yes | storage policy |
| CreateNewMap / StartupZero | Core initialization contract | `test_slam_initialization_contract`, Simulation Mapping smoke | yes | none |
| LoadExistingMap / ConfiguredPose | Core initialization and frame state | `test_sparse_slam_runtime_core_initialization`, Simulation Localization smoke | yes | map deployment |
| Startup relocalization | Core relocalization manager | `test_sparse_slam_runtime_startup_relocalization`, `test_m3d3b_startup_relocalization_simulation` | yes | live scan tuning |
| Symmetric relocalization rejection | Core relocalizer gates | `test_sparse_multi_tof_relocalizer`, rejection formal scenario | yes | field environment validation |
| Localization Health | Core health tracker | `test_localization_health_tracker` | yes | real health thresholds |
| Lost Recovery | Core local/global recovery | `test_sparse_slam_runtime_lost_recovery`, `test_m3d3b_lost_recovery_simulation` | yes | real failure injection |
| Recovery failure Stop | Application motion boundary | lost-recovery tests and motion test | yes | real stop transport |
| Frontier generation | `SparseFrontierPlanner` through controller | `test_sparse_frontier_astar`, `test_frontier_goal_quality` | yes | none |
| A* navigation | `BoundedAStar` and immutable navigation view | `test_sparse_frontier_astar`, formal exploration | yes | none |
| Safe waypoint tracking | `SafeWaypointTracker` | `test_safe_waypoint_tracker`, formal exploration | yes | none |
| Failure memory/revision revalidation | exploration controller/tracker | M3-E1 exploration tests | yes | none |
| Obstacle stop and termination confirmation | controller and simulation adapter | `test_robot_slam_application_exploration`, formal `collision=0` and `no_reachable_frontier` | yes | real ToF safety validation |
| Motion TTL/stale/deadman | Application motion boundary safety | `test_robot_slam_application_motion`, BL4820 safety tests | yes | real feedback validation |
| Motion writer/transport | Application motion boundary, writer, software transport | `test_motion_command_writer`, `test_software_motion_command`, `test_software_motion_transport_contract_checker` | yes | Production transport |
| Simulation adapter | `SimSensorPort` / `SimMotionPort` | `test_sim_sensors`, `test_sim_motion_port`, ground-truth isolation/oracle tests | yes | none |
| Replay adapter | `MultiTofReplaySensorPort` | replay codec/port tests and `test_robot_slam_application_replay` | yes | none |
| Replay no-motion | Application no-motion transport | `test_robot_slam_application_motion`, formal replay smoke | yes | none |
| Real fail-closed | formal `real_main` gate and real stubs | `test_real_adapter_factory`, `test_real_adapter_readiness_checker`, real unavailable smoke | yes | implement real sensor adapter |
| Ground-truth isolation | Simulation reporting boundary only | `test_m3d1_ground_truth_isolation`, `test_m3e_exploration_oracle_boundary`, structure audit | yes | none |
| Deterministic repeatability | Simulation adapter/Core/controller | `test_m3e_exploration_determinism`, repeated formal exploration reports | yes | none |

## Formal evidence baseline

The trusted Simulation Exploration baseline is four completed goals, zero
collisions, termination `no_reachable_frontier`, map revision 2041, 2876 known
cells, and 31 keyframes. Its report also records one Application/Core, one
odometry estimator, one sparse backend, and `ground_truth_isolation_verified`.
Mapping and Localization formal runs use the same Application and report map
revision/cell behavior; Localization leaves the loaded map revision unchanged.

The remaining production work is deliberately outside this consolidation:
implementing and calibrating a real sensor adapter, implementing the production
motion transport, and validating field parameters. Until those exist,
`sensor_source=real` remains fail-closed and no UART motor write is enabled.
