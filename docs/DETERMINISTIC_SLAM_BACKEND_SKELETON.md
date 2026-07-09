# Deterministic SLAM Backend Skeleton

M3-B3 adds a deterministic SLAM backend skeleton for the `SlamBackendBinding` interface. It is not production SLAM, does not call `Localizer`, does not call `mapping_supervisor`, and does not modify `map_quality.hpp` or the real map quality scoring path.

The skeleton accepts `RobotSlamSensorSnapshot` through `SlamBackendInputFrame`, evaluates the ToF scan with deterministic contract checks, updates a skeleton quality tracker, and returns `SlamBackendUpdateResult` plus `RobotSlamMapQuality`.

The reported `coverage_ratio` is a skeleton proxy based on accepted scan count. The reported `yaw_coverage_ratio` is a skeleton proxy based on deterministic yaw bins. Neither value is real occupancy-map coverage.

This backend does not write map files. `save_map` remains disabled by default and returns a stable failure message. The app runtime does not create this backend by default.

Passing M3-B3 proves the replay-to-backend interface chain can run deterministically. It does not prove real SLAM readiness, real map quality, real localization, or live robot readiness.
