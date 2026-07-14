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
