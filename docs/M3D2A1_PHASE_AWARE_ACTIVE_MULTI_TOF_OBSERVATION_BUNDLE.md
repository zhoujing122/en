# M3-D2A1 Phase-Aware Active Multi-ToF Observation Bundle

This stage adds the bounded active multi-ToF observation bundle contracts used by the unified SparseSlamRuntimeCore path. The bundle stores native front/left/right scalar ToF frames, each route effective timestamp, each route measurement-time odom pose, and each route measurement-time map pose. It does not store ground truth, command-derived pose, fake beams, legacy point clouds, FilteredTofSample, ChunkedGrid state, matcher inputs, or keyframes.

## Bundle Contract

The active bundle is bounded by max snapshots, max rays, max duration, and max snapshot gap. Append is transactional: validation failure leaves accepted frames unchanged. Hit and explicit NoReturn rays are matchable; Invalid and Unspecified rays are counted but are not matchable. The scalar return semantics are preserved and Invalid is never converted to NoReturn.

The frozen bundle owns an immutable frame vector and a fixed bundle id, reference map revision, reference map cell count, map_from_odom_at_start, ray counts, and yaw coverage summary. Freezing requires enough snapshots, matchable rays, yaw coverage, and an unchanged reference map revision. This stage does not implement local matching, keyframes, pose correction, map save/load, relocalization, hardware, or motion.

## Phase Controller and Settle Gate

The phase-aware controller defines the transitional collection lifecycle: Idle, CollectingDuringMotion, WaitingForMotionSettle, FrozenReady, and Aborted. Only Idle allows normal map commit. BeginCollection immediately blocks map commit; EndMotionAndWaitForSettle keeps collection active while the settle gate uses canonical wheel and IMU samples to prove stability.

MotionSettleGate consumes only wheel freshness, wheel linear speed, wheel angular speed, IMU freshness, IMU yaw rate, and sensor timestamps. It does not read command speed, command-completed flags, target angle, or wall-clock sleep. Stale samples, non-finite values, speed threshold violations, timestamp rollback, and sample gaps reset or reject stability deterministically.

The reference map guard records map revision and cell count at BeginCollection. Any revision or cell count change during active collection aborts the bundle and keeps map commit blocked until explicit discard.

## Runtime Core Integration

`SparseSlamRuntimeCore` owns the phase-aware controller, active builder, frozen bundle state, settle gate, map revision, estimator, timed pose buffer, measurement pose resolver, backend adapter, and sparse backend. `SparseShadowRuntime` remains a wrapper that constructs the deterministic sensor source, constructs the core, calls `initialize`, calls `step`, and writes the report. Future production wiring must replace the sensor source and reuse the same core.

The only sparse map commit gate is the active bundle state: Idle allows the existing D2A0 backend update path; CollectingDuringMotion, WaitingForMotionSettle, FrozenReady, and Aborted block `SlamBackendMapPortAdapter::integrate_map_update` before backend attempt counts increase. Odometry and the timed pose buffer continue to update while map commit is blocked. WaitingForMotionSettle continues native multi-ToF collection and uses `MotionSettleGate` before freezing.

`current_map_revision` increases only after an accepted idle sparse backend update. BeginCollection records `reference_map_revision`, `reference_map_cell_count`, and `map_from_odom_at_start`; active/frozen/aborted states verify the revision and cell count remain unchanged. The frozen bundle is not committed to the sparse map in this stage, so the reference map cannot contain the bundle being prepared for the future matcher.
