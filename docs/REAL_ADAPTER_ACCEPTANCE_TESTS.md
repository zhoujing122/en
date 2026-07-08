# Real Adapter Acceptance Tests

A real adapter candidate must pass shadow acceptance before any live test:

1. Sensor port is present and ready.
2. Motion port is present and ready.
3. Map port is present and ready.
4. ToF frame contract passes.
5. At least one of IMU or wheel odom passes when required.
6. Map quality contract passes.
7. Readiness checker returns `ShadowReady`.
8. Shadow acceptance is complete before any live path is considered.
9. M2-B3 software transport acceptance and M2-B1 lifted direction probe are
   still required before live movement.

M3-A1 acceptance verifies adapter contract shape only. It does not prove a real
sensor driver, real chassis transport, or real TTL stop implementation.
