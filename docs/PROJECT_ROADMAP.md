# Project Roadmap

- M3-C3: complete. Three-ToF replay log pipeline is in place.
- M3-C3.1: complete. Real scalar ToF protocol, range, FOV, timestamp, and freshness semantics are aligned.
- M3-C4: complete. Scalar three-ToF replay is connected to the full fake autonomous pipeline through `RobotSlamSensorPort`, while the deterministic backend still consumes only the legacy front scalar projection.
- M3-D0: complete. Deterministic closed-loop 2D robot motion and sensor simulation is in place.
- M3-D1: algorithm complete. Invalid / Hit / NoReturn scalar ToF semantics, Wheel/IMU dead reckoning, and the lightweight sparse three-ToF occupancy backend are implemented.
- M3-D1.1: complete. The executable has a default-off Sparse Shadow runtime wiring path for explicit predicted map pose handoff into the sparse backend.
- M3-D2A0: next stage. Add initialization contracts, map/odom frames, and a timed odom pose buffer.
- M3-D2: later stage. Add sparse scan matching, pose correction, and keyframes after D2A0.
- M3-E: future. Add Frontier and A* exploration.
- M3-F: complete simulation acceptance.

M3-C4 is not real hardware enablement, production SLAM, native three-ToF map fusion, real map writing, pose writeback, or autonomous exploration.
