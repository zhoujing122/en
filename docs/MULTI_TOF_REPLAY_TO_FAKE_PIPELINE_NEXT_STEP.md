# Multi-ToF Replay To Fake Pipeline Next Step

M3-C3 stops at replay-to-snapshot. M3-C4 should connect the generated Multi-ToF snapshots to the existing fake/replay autonomous SLAM product flow.

M3-C4 should still remain fake/replay only: no real ToF driver, no `/dev` access, no real motion, no real map writes, and no pose writeback.
