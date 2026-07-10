# Multi-ToF Next Steps

M3-C0 stops at raw data contract validation. Planned follow-up stages are:

- M3-C1: three-ToF plus IMU/Wheel sync checker.
- M3-C2: multi-ToF snapshot builder.
- M3-C3: multi-ToF replay/log format and codec.
- M3-C4: three-ToF fake pipeline.
- M3-C5: real adapter skeleton, still fail-closed before hardware sign-off.

None of these follow-up behaviors are implemented by M3-C0.

## Current Status After M3-C1

M3-C1 implements the offline three-ToF plus IMU/Wheel synchronization checker. M3-C2 remains the next step for a multi-ToF snapshot builder, and M3-C3 remains the future replay/log stage.

M3-C2 status: Multi-ToF snapshot construction is implemented. Replay/log remains deferred to M3-C3.

M3-C3 status: 3-ToF replay/log is implemented for in-memory logs. The next staged work is M3-C4 fake/replay full pipeline integration.
