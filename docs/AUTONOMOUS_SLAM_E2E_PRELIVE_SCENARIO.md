# Autonomous SLAM E2E Pre-live Scenario

M3-A4 defines an end-to-end pre-live autonomous SLAM scenario. It is not a toy test and it is not live motion. It connects sensor replay, real adapter contract checking, SLAM backend binding, RobotSlamMapPort, the pre-live runner, AutonomousSlamCoordinator, policy, shadow motion, gate, and report generation.

Passing this scenario means the end-to-end pre-live shadow integration path is usable. It does not prove real robot motion, real ToF/IMU/Wheel driver readiness, or real SLAM map quality. Future live work should replace only RealTofImuWheelSensorPort, RealSoftwareMotionPort, and RealSlamBackendBinding while leaving the coordinator, policy, pre-live runner, and E2E gate logic intact.

After this passes, the next step is still lifted direction-probe preparation, not ground motion.
