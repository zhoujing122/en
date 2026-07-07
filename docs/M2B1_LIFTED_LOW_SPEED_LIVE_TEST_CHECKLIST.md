# M2-B1 Lifted Low-Speed Live Test Checklist

Do not enter any M2-B1 live step unless Xiaoen wheels are lifted or otherwise free from the ground. Ground motion is not allowed in these stages.

## Stage 1: LiftedDirectionProbe

Use this stage only for the first lifted low-speed direction convention check. Direction convention may be pending in this stage.

1. Xiaoen wheels are lifted or free from the ground.
2. Operator is present.
3. Emergency stop is available and reachable.
4. Software transport shadow mode has been verified.
5. STOP command has been verified.
6. TTL timeout STOP has been verified.
7. ACK / rejected semantics have been verified.
8. Encoder read is OK.
9. Encoder latency is OK.
10. Encoder pair skew is OK.
11. Motor status is zero.
12. Current is within limit.
13. Obstacle path is clear for a lifted-wheel test.
14. `speed_normalized <= 0.03`.
15. `duration_s <= 0.30`.
16. Send STOP.
17. Send TURN_LEFT for 0.2 to 0.3 s.
18. Send STOP.
19. Send TURN_RIGHT for 0.2 to 0.3 s.
20. Send STOP.
21. Record whether left negative / right positive is truly TURN_LEFT.
22. Record whether left positive / right negative is truly TURN_RIGHT.
23. If direction is reversed, do not change hardware; fix software mapping or configuration first.
24. Do not put Xiaoen on the ground until direction convention is confirmed.

## Stage 2: ConfirmedLiftedLive

Use this stage only after the direction convention has been confirmed in Stage 1.

1. Direction convention is confirmed.
2. Xiaoen wheels are still lifted or free from the ground.
3. Operator is present.
4. Emergency stop is available and reachable.
5. Encoder read, latency, pair skew, status, and current gates are still OK.
6. `speed_normalized <= 0.05`.
7. `duration_s <= 0.50`.
8. Send STOP before every non-stop command.
9. Test TURN_LEFT for no more than 0.5 s, then STOP.
10. Test TURN_RIGHT for no more than 0.5 s, then STOP.
11. Do not run ground motion in M2-B1.
12. Do not enter any later ground-motion stage until this checklist passes and a separate ground-motion checklist exists.

## Algorithm Motion API Link

Direction probe and confirmed lifted live procedures should use the high-level command definitions in `docs/ALGORITHM_MOTION_API.md`. `DirectionProbeLeft` / `DirectionProbeRight` are allowed only for lifted, low-speed direction verification before any grounded motion.

## M2-B3 Prerequisite

Do not enter lifted live testing until the software transport passes the M2-B3 shadow acceptance contract. M2-B3 itself does not move the robot and does not validate real TTL stopping on hardware.
