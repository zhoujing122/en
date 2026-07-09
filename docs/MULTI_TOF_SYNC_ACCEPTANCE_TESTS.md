# Multi-ToF Sync Acceptance Tests

M3-C1 acceptance covers deterministic offline sync cases:

- valid synced packet passes.
- front-left sync too large fails.
- front-right sync too large fails.
- left-right sync too large fails.
- IMU sync too large fails.
- Wheel sync too large fails.
- missing left fails under `RequireAll`.
- missing left passes degraded under `AllowMissingOne`.
- missing IMU fails when IMU is required.
- missing Wheel fails when Wheel is required.

No real hardware, SDK, serial device, filesystem map, or motion transport is used.
