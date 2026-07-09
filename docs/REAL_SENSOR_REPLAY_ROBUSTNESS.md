# Real Sensor Replay Robustness

M3-B2.1 hardens the offline real sensor replay/log layer. It does not read real hardware, does not read SDK data, does not access /dev, and does not start from app runtime by default.

Replay parse errors are preserved as InvalidRecord records. They are not converted to comments and are not silently skipped. Invalid numeric values are rejected by strict parsing, empty tof_ranges is rejected, and comment-only logs are not considered ready because they contain no Packet records.

RealSensorReplayPort is ready only when the log is non-empty, contains at least one Packet record, and has no fatal InvalidRecord when reject_invalid_records is true. When a packet fails contract validation with fail_on_contract_error=true, latest_snapshot returns an empty snapshot instead of reusing a stale last valid snapshot.

Replay validation defaults to record_packet_time. This keeps real logs from being rejected as stale or future merely because an external test clock is far from the recorded packet time. ExternalNow and record_sensor_max_time remain explicit test modes.

ToF and Wheel timestamps remain request-window estimates. Replay logs must keep request_start_s, response_received_s, estimated_sample_time_s, and request_latency_s; they must not collapse ToF/Wheel timing to one hardware capture timestamp.

Passing this stage only means the offline replay/log layer is stricter and more diagnosable for later SLAM backend regression. It does not prove real hardware readiness or live motion readiness.
