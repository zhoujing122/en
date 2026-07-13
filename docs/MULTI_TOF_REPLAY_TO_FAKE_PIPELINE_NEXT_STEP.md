# Multi-ToF Replay To Fake Pipeline

This document is superseded by `docs/M3C4_SCALAR_MULTI_TOF_FAKE_PIPELINE.md` for the current M3-C4 implementation details.

M3-C4 proves that scalar three-ToF replay data enters the complete fake autonomous pipeline:

```text
9-byte scalar replay
-> codec
-> contract
-> sync
-> snapshot
-> RobotSlamSensorPort
-> FullAutonomousSlamRunner
-> AutonomousSlamCoordinator
-> legacy scalar projection
-> DeterministicSlamBackendBinding
-> fake motion/report
```

M3-C4 remains fake/replay only: no real ToF driver, no `/dev` access, no real UART, no real motion, no real map writes, and no pose writeback.

M3-C4 proves three scalar ToF readings are preserved through the fake pipeline, but the current `DeterministicSlamBackendBinding` still uses only the front ToF legacy single-scalar projection. Native three-ToF map fusion is not implemented in M3-C4.
