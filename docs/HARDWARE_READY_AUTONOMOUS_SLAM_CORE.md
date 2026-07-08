# Hardware-Ready Autonomous SLAM Core

M3-A0 is not a toy synthetic test. The goal is to make the autonomous SLAM
core look like the code that will run with a real xiaoen robot, while keeping
all live hardware paths disabled until the software-side robot interfaces are
implemented and validated.

The core design is ports plus adapters:

- `RobotSlamSensorPort` supplies ToF, IMU, and wheel/odom snapshots.
- `RobotSlamMotionPort` accepts `AlgorithmMotionCommand` and returns motion
  feedback.
- `RobotSlamMapPort` integrates snapshots, reports map quality, and saves maps.
- `RobotSlamTimePort` supplies deterministic or real time.

`AutonomousSlamCoordinator` depends only on these ports. It does not include a
real ToF driver, a real IMU driver, a wheel driver, a true chassis transport, or
any robot-specific runtime API. Current tests use fake and shadow ports.

Future real-robot integration should add adapters such as:

- `RealTofSensorPort`
- `RealImuWheelSensorPort`
- `RealSoftwareMotionPort`
- `RealMapPort`

Those adapters should translate real robot data into the port data structures.
They should not require changes to `AutonomousSlamCoordinator`,
`AutonomousSlamPolicy`, or the state machine.

Current M3-A0 behavior:

- Only stop and low-speed active-scan rotation commands are generated.
- Forward and backward motion remain defined at the algorithm API layer but
  disabled by default.
- The coordinator is not created by the app runtime by default.
- No real chassis transport is provided.
- No real ToF, IMU, or wheel adapter is provided.

Before any live stage, the robot must still pass:

- software transport shadow acceptance
- lifted direction probe
- STOP, emergency stop, and TTL validation
- low-speed lifted scan validation

M3-A0 can prove:

- the autonomous SLAM main state machine is complete enough for adapter
  integration
- sensor input, map quality, and motion output are abstracted
- real-robot integration has a clear path

M3-A0 cannot prove:

- the real robot can move
- chassis direction mapping is correct
- real ToF data quality is sufficient
- real map quality is acceptable
- the real software layer has implemented TTL stop

## M3-A1 Real Adapter Contracts

M3-A1 adds real adapter contract checkers, readiness checks, replay adapters, and acceptance tests. These define the data quality and readiness rules for future real ToF, IMU, wheel, map, and motion adapters without implementing any real driver. See `docs/REAL_ADAPTER_CONTRACTS.md`, `docs/REAL_ADAPTER_ACCEPTANCE_TESTS.md`, and `docs/REAL_ADAPTER_HANDOFF_CHECKLIST.md`.

M3-A2 adds docs/PRELIVE_AUTONOMOUS_SLAM_INTEGRATION.md, which wires the hardware-ready ports, contracts, coordinator, policy, shadow motion, gate, and report into a pre-live integration runner.

M3-A3 adds docs/SLAM_BACKEND_MAP_PORT_BINDING.md, defining how future SLAM backends bind to RobotSlamMapPort without changing the coordinator.
