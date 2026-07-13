# Multi-ToF Replay To Fake Pipeline Next Step

M3-C3 stops at replay-to-snapshot. M3-C4 should connect the generated Multi-ToF snapshots to the existing fake/replay autonomous SLAM product flow.

M3-C4 should still remain fake/replay only: no real ToF driver, no `/dev` access, no real motion, no real map writes, and no pose writeback.

## M3-C3.1 Protocol Alignment Note

M3-C3.1 replaces the M3-C three-ToF replay contract with scalar single-point ToF readings backed by the confirmed 9-byte payload: 48-bit echo tag, `distance_mm`, and `confidence`. The echo tag is retained for correlation/debug only and is not used as measurement time. Synchronization uses request-window midpoint timestamps. The default full ToF FOV is 1.6 degrees and must not be expanded into synthetic ranges or point clouds. Legacy `snapshot.tof` is only a one-range compatibility projection for old fake backends. This stage does not connect real ToF/UART hardware, enable real motion, write real maps, perform pose writeback, or implement true three-ToF SLAM fusion.
