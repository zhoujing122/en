# Full Pipeline Trace And Phase-Aware Replay

M3-B5 changes the fake/replay full autonomous SLAM runner so it does not advance replay on every loop unconditionally.

Replay frames should be consumed when the coordinator phase needs fresh sensor data: the first step, Idle, WaitingForSensors, Initializing, Mapping, and IntegratingScan. Replay frames are skipped during NeedActiveScan, SendingMotionCommand, WaitingMotionSettle, Completed, and Fault. This prevents motion command and settle phases from silently consuming sensor log records.

The runner records a structured trace with step start, sensor consumed/skipped, backend update/reject, coordinator update, motion command, map quality, fake map build/save, completed, and fault events. The trace is in memory only and is intended for debugging fake pipeline regressions.
