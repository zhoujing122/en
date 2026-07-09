# Fake To Real Replacement Manifest

M3-B7 records the adapter replacements required before live operation. RealSensorReplayPort must be replaced by RealTofImuWheelSensorPort. FullAutonomousSlamFakeMotionPort must be replaced by RealSoftwareMotionPort. DeterministicSlamBackendBinding must be replaced by RealSlamBackendBinding. FakeMapStorage must be replaced by real map storage. FakeRelocalizationBinding must be replaced by reviewed real relocalization.

Fake map artifacts are metadata only; real hardware integration requires real occupancy/grid map artifacts. Fake pose candidates are report-only and must remain gated until pose writeback is reviewed.
