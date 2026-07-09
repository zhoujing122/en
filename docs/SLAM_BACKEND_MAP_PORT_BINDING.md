# SLAM Backend Map-Port Binding

M3-A3 defines the contract between a future SLAM backend and RobotSlamMapPort. It does not implement a real SLAM backend, does not change Localizer, mapping_supervisor, or map_quality behavior, and does not add hardware access.

The binding flow is:

1. RobotSlamSensorSnapshot is wrapped into SlamBackendInputFrame.
2. SlamBackendContractChecker validates the input frame.
3. SlamBackendMapPortAdapter calls a SlamBackendBinding implementation.
4. The backend returns SlamBackendUpdateResult.
5. The result and RobotSlamMapQuality are checked.
6. AutonomousSlamCoordinator continues to depend only on RobotSlamMapPort.

Future RealSlamBackendBinding can wrap the actual SLAM backend and map quality source. ReplaySlamBackendBinding exists only for tests and pre-live contract verification. Passing M3-A3 acceptance means the binding contract is coherent; it does not prove a real map backend is usable.
M3-A4 adds `docs/AUTONOMOUS_SLAM_E2E_PRELIVE_SCENARIO.md` for the end-to-end pre-live shadow scenario across replay sensors, SLAM backend binding, map port, pre-live runner, coordinator, policy, and shadow motion.

## M3-B3 Deterministic Backend Skeleton

M3-B3 adds a deterministic `SlamBackendBinding` implementation used only for offline contract and replay-to-map regression. It proves the binding path can return `SlamBackendUpdateResult` and `RobotSlamMapQuality`; it is not production SLAM and does not write real maps.
