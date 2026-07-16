# Project Roadmap

- M3-C3: complete. Three-ToF replay log pipeline is in place.
- M3-C3.1: complete. Real scalar ToF protocol, range, FOV, timestamp, and freshness semantics are aligned.
- M3-C4: complete. Scalar three-ToF replay is connected to the full fake autonomous pipeline through `RobotSlamSensorPort`, while the deterministic backend still consumes only the legacy front scalar projection.
- M3-D0: complete. Deterministic closed-loop 2D robot motion and sensor simulation is in place.
- M3-D1: complete. Invalid / Hit / NoReturn scalar ToF semantics, Wheel/IMU dead reckoning, and native sparse three-ToF occupancy backend are in place.
- M3-D2: complete. Bounded sparse ToF local matching, map/odom pose correction, and keyframe transactions are implemented for deterministic simulation.
- M3-E: next stage. Add Frontier and A* exploration.
- M3-F: follow-up. Complete simulation acceptance.

M3-C4 is not real hardware enablement, production SLAM, native three-ToF map fusion, real map writing, pose writeback, or autonomous exploration.
