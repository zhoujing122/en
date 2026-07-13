# Project Roadmap

- M3-C3: complete. Three-ToF replay log pipeline is in place.
- M3-C3.1: complete. Real scalar ToF protocol, range, FOV, timestamp, and freshness semantics are aligned.
- M3-C4: complete. Scalar three-ToF replay is connected to the full fake autonomous pipeline through `RobotSlamSensorPort`, while the deterministic backend still consumes only the legacy front scalar projection.
- M3-D0: current stage in this working tree. Deterministic closed-loop 2D robot motion and sensor simulation is being added.
- M3-D1: next stage. Add the lightweight sparse three-ToF occupancy backend.
- M3-D2: add sparse scan matching, pose correction, and keyframes.
- M3-E: add Frontier and A* exploration.
- M3-F: complete simulation acceptance.

M3-C4 is not real hardware enablement, production SLAM, native three-ToF map fusion, real map writing, pose writeback, or autonomous exploration.
