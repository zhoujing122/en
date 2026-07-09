# Multi-ToF Acceptance Tests

M3-C0 acceptance covers only offline raw contract behavior. It checks:

- valid three-ToF packet passes.
- missing left ToF fails when left is required.
- duplicate frame ID fails.
- duplicate mount ID fails.
- invalid mount yaw fails.
- empty ranges fail.
- excessive request latency fails.

The acceptance report states that three-ToF pairwise sync is deferred to M3-C1 and multi-ToF snapshot building is deferred to M3-C2. No real ToF/IMU/Wheel device is read.
