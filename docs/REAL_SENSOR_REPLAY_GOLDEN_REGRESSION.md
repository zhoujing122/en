# Real Sensor Replay Golden Regression

M3-B2.1 adds a golden replay regression harness for offline log behavior. The default cases cover a valid short log, request latency rejection, sync-too-large rejection, parse error detection, and comment-only log rejection.

The harness uses inline deterministic replay logs. It does not read files by default, does not read ToF/IMU/Wheel devices, does not connect to a real SLAM backend, and does not send motion commands.

Expected behavior is explicit: valid logs must pass, invalid latency and sync logs must fail, parse errors must produce InvalidRecord, and comment-only logs must be rejected for having no Packet records.

ToF and Wheel timestamps are request-window estimates. Golden cases must preserve request_start_s, response_received_s, estimated_sample_time_s, and request_latency_s so later map backend regression uses the same timing contract as real captures.

This harness is a skeleton for future real-data regression. When real captures exist, add golden log cases here before connecting RealSlamBackendBinding to live data. Passing this harness does not prove real hardware readiness.
