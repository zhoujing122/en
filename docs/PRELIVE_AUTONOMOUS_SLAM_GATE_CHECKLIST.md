# Pre-live Autonomous SLAM Gate Checklist

The M3-A2 gate blocks live progression unless the shadow integration report satisfies the configured conditions.

Required checks:

1. Real adapter readiness passes.
2. Sensor snapshot contract passes.
3. Map quality contract passes.
4. Adapter acceptance passes when required.
5. Coordinator does not enter Fault.
6. No motion rejection is seen when required.
7. Stop command is seen when required.
8. Forward/backward commands are not generated.
9. Hardware write remains disabled.
10. Production interface remains disabled.
11. Shadow acceptance has passed.
12. Live work still requires manual emergency stop, lifted robot, and direction probe confirmation.

Passing this gate is only permission to prepare lifted low-speed direction probe validation. It is not permission for grounded autonomous movement.
