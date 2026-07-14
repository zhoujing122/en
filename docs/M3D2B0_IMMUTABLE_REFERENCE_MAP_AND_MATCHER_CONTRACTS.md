# M3-D2B0 Immutable Sparse Reference Map and Matcher Contracts

## Reference map snapshot

SparseOccupancyGridSnapshot is the immutable value snapshot of the live sparse
occupancy grid. Capture copies the grid once after a bundle freezes. It retains
the runtime map revision, grid resolution, evidence thresholds, raw signed
evidence, deterministic SparseGridCellKey order, classification counts and
bounds.

The snapshot exposes only const cell storage and read-only key/world queries.
Missing cells are Unknown; queries never insert cells. Capture is transactional
and fails before producing a valid snapshot when the grid is invalid or the
configured hard cell limit would be exceeded.

Begin collection records revision and cell count but does not copy the map.
Collecting and settle phases keep the map commit gate closed. The runtime will
capture exactly once after freeze and will reject a revision or cell-count
mismatch. A snapshot is an in-memory matcher reference, not map persistence,
and contains no bundle observations or ground-truth state.

## Local matcher contracts

SparseTofLocalMatchInput combines one immutable Frozen bundle, one immutable
reference snapshot, the expected and current map revisions, the predicted
map_T_odom search center and the collection-start map_T_odom. Shared ownership
is const-only and has no live grid, command, Localizer or ground-truth pointer.

The validator rejects missing or mutable-equivalent inputs, incomplete bundles,
invalid route timestamps or poses, sparse reference evidence, revision mismatch,
invalid predictions and early request timestamps. Search configuration is
bounded and defaults to YawOnly with zero translation range. FullSE2 is only an
expressible future mode in B0.

SparseTofLocalMatchResult defaults to NotRun. Scores, proposed transforms and
correction deltas are optional and absent until a matcher actually executes.
The B0 placeholder returns NotImplemented with matcher_executed=false and zero
evaluated candidates; SparseSlamRuntimeCore does not call it.

## Runtime preparation

`SparseSlamRuntimeCore` prepares the matcher input only on the transition from
WaitingForMotionSettle to FrozenReady. It first revalidates the begin-time map
revision and cell count, captures one immutable snapshot, copies the frozen
bundle into const shared ownership, reads the current map_T_odom prediction and
validates the complete input. Collecting and Waiting never capture the map.

A preparation failure leaves no ready snapshot or partial input, changes the
active phase to Aborted and keeps the map commit gate closed. Explicit Discard
clears only the frozen matcher foundation state; it preserves the live sparse
map, frame state, estimator, initialization and timed pose buffer. B0 never
calls the matcher, generates candidates, scores observations, applies a
correction or creates a keyframe.
