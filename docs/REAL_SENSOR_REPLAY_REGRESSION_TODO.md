# Real Sensor Replay Regression TODO

Future real-data regression work must define:

1. Real robot capture format and ownership.
2. ToF/Wheel request window recording owner.
3. IMU timestamp source and units.
4. Golden replay cases for valid, stale, high-latency, and sync-failure captures.
5. Map backend replay chain using `RealSensorReplayPort`.
6. Regression report format.
7. Debug ownership when failures split across sensor, SLAM, map quality, and policy layers.

Forward/backward live movement remains out of scope for replay regression.
