# Multi-ToF Sync To Snapshot Next Step

M3-C1 stops after synchronization validation. M3-C2 should add a multi-ToF snapshot builder that converts a synchronized `MultiTofRawPacket` into a future snapshot representation without breaking the existing single-ToF `RobotSlamSensorSnapshot` path.

Open M3-C2 items:

- define snapshot representation for front/left/right ToF.
- preserve each ToF request-window timing in diagnostics.
- define how synchronized multi-ToF data enters deterministic backend tests.
- keep app runtime disabled by default.
