# Fake To Real Replacement Manifest

M3-B7 records the adapter replacements required before live operation. RealSensorReplayPort must be replaced by RealTofImuWheelSensorPort. FullAutonomousSlamFakeMotionPort must be replaced by RealSoftwareMotionPort. DeterministicSlamBackendBinding must be replaced by RealSlamBackendBinding. FakeMapStorage must be replaced by real map storage. FakeRelocalizationBinding must be replaced by reviewed real relocalization.

Fake map artifacts are metadata only; real hardware integration requires real occupancy/grid map artifacts. Fake pose candidates are report-only and must remain gated until pose writeback is reviewed.

## M3-C0 Multi-ToF Contract

The fake-to-real replacement plan now includes a future three-ToF sensor adapter. M3-C0 only defines raw front/left/right ToF contract data with yaw `0/+90/-90`; it does not replace `RealSensorReplayPort` or build multi-ToF snapshots yet.
