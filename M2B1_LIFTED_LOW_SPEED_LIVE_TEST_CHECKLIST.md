# M2-B1 Lifted Low-Speed Live Test Checklist

Do not enter M2-B1-live until every item below is true.

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
13. `max_speed_normalized <= 0.05`.
14. `max_duration_s <= 0.5`.
15. Send STOP, then TURN_LEFT for 0.3 s, then STOP.
16. Send STOP, then TURN_RIGHT for 0.3 s, then STOP.
17. Confirm left negative / right positive is truly TURN_LEFT.
18. Confirm left positive / right negative is truly TURN_RIGHT.
19. If direction is reversed, do not change hardware; fix software mapping or
    configuration first.
20. Do not put Xiaoen on the ground until direction convention is confirmed.
21. Do not enter M2-B1-live until this checklist passes.
