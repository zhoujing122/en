# Live Handoff Readiness Checklist

Live handoff is blocked until all required evidence exists.

- RealTofImuWheelSensorPort is implemented and passes the M3-A1 contract.
- RealSoftwareMotionPort is implemented and passes the M2-B3 transport
  acceptance.
- RealSlamBackendBinding is implemented and passes the M3-A3 backend
  acceptance.
- M3-A4 E2E pre-live passes.
- STOP, E-STOP, and TTL behavior are measured on the vehicle.
- Lifted direction probe passes.
- Operator is present.
- Emergency stop is available.
- Wheels are lifted or free to turn.
- Forward and backward remain disabled.
- Project manager confirms the test window and safety owner.

M3-B0 cannot return LiveReady. It can only report blocked or stub-only
handoff states until real hardware verification exists.
