# Pre-live Autonomous SLAM Integration

M3-A2 defines the pre-live autonomous SLAM integration flow. It is not a toy test, and it is not live robot motion. The flow connects readiness, real adapter contracts, adapter acceptance, the autonomous SLAM coordinator, active scan policy, shadow motion, a gate, and a text report.

The runner validates only pre-live shadow integration. Passing means the current ports and adapters are coherent enough to prepare for lifted low-speed direction probe work. Passing does not prove real robot motion, real ToF readiness, real IMU readiness, real wheel odometry readiness, map quality on hardware, or a real TTL stop implementation.

The flow is:

1. Check RobotSlamSensorPort, RobotSlamMotionPort, RobotSlamMapPort, and RobotSlamTimePort readiness.
2. Check RobotSlamSensorSnapshot and RobotSlamMapQuality with RealAdapterContractChecker.
3. Run RealAdapterAcceptanceRunner once.
4. Run AutonomousSlamCoordinator steps with shadow or fake ports.
5. Let AutonomousSlamPolicy request active scan rotation only when map quality is poor.
6. Record AlgorithmMotionCommand and SoftwareMotionCommand projections.
7. Evaluate PreLiveAutonomousSlamGate.
8. Emit PreLiveIntegrationReport text.

Future live integration should replace ports/adapters only: RealTofImuWheelSensorPort, RealSoftwareMotionPort, and RealMapPort. AutonomousSlamCoordinator and AutonomousSlamPolicy should remain unchanged unless the contract itself changes.
