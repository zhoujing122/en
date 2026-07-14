# M3-D2A1 Phase-Aware Active Multi-ToF Observation Bundle

This stage adds the bounded active multi-ToF observation bundle contracts used by the unified SparseSlamRuntimeCore path. The bundle stores native front/left/right scalar ToF frames, each route effective timestamp, each route measurement-time odom pose, and each route measurement-time map pose. It does not store ground truth, command-derived pose, fake beams, legacy point clouds, FilteredTofSample, ChunkedGrid state, matcher inputs, or keyframes.

## Bundle Contract

The active bundle is bounded by max snapshots, max rays, max duration, and max snapshot gap. Append is transactional: validation failure leaves accepted frames unchanged. Hit and explicit NoReturn rays are matchable; Invalid and Unspecified rays are counted but are not matchable. The scalar return semantics are preserved and Invalid is never converted to NoReturn.

The frozen bundle owns an immutable frame vector and a fixed bundle id, reference map revision, reference map cell count, map_from_odom_at_start, ray counts, and yaw coverage summary. Freezing requires enough snapshots, matchable rays, yaw coverage, and an unchanged reference map revision. This stage does not implement local matching, keyframes, pose correction, map save/load, relocalization, hardware, or motion.

## Phase Controller and Settle Gate

The phase-aware controller defines the transitional collection lifecycle: Idle, CollectingDuringMotion, WaitingForMotionSettle, FrozenReady, and Aborted. Only Idle allows normal map commit. BeginCollection immediately blocks map commit; EndMotionAndWaitForSettle keeps collection active while the settle gate uses canonical wheel and IMU samples to prove stability.

MotionSettleGate consumes only wheel freshness, wheel linear speed, wheel angular speed, IMU freshness, IMU yaw rate, and sensor timestamps. It does not read command speed, command-completed flags, target angle, or wall-clock sleep. Stale samples, non-finite values, speed threshold violations, timestamp rollback, and sample gaps reset or reject stability deterministically.

The reference map guard records map revision and cell count at BeginCollection. Any revision or cell count change during active collection aborts the bundle and keeps map commit blocked until explicit discard.
