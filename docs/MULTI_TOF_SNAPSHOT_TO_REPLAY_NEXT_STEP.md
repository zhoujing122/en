# Multi-ToF Snapshot To Replay Next Step

M3-C2 stops at in-memory snapshot construction. M3-C3 should add Multi-ToF replay/log support.

Expected M3-C3 work:

- encode front, left, and right ToF frames in replay logs.
- preserve each request window: request_start, response_received, estimated_sample_time, and request_latency.
- replay into `MultiTofRawPacket`.
- run raw contract, sync checker, and snapshot builder from replay data.
- keep legacy single-ToF replay compatibility.
- do not connect real hardware until a later real adapter stage.
