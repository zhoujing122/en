# M3-D3A Full ToF Extrinsics and Sparse Map Lifecycle

## Geometry contract

The sparse runtime owns one configured `PlanarThreeTofExtrinsics` value. For
each route, geometry is computed as:

`map_T_sensor(t) = map_T_odom * odom_T_base(t) * base_T_sensor`

`SparseTofObservationBuilder` is the only component that applies
`base_T_sensor`. Normal mapping, YawOnly candidate scoring, and corrected
Keyframe reprojection all call that builder. The legacy mount-yaw-only path is
explicit compatibility behavior and is not accepted for persisted sparse maps.

## Artifact contract

Format version 1 is a deterministic text artifact. It stores map and run IDs,
the complete sparse-grid geometry/evidence policy, revision, all cells with raw
evidence, all three planar ToF extrinsics, and wheel/encoder geometry. Cells use
the grid's stable key ordering. A deterministic FNV-1a checksum covers the
canonical payload.

Save captures one immutable snapshot in stable Idle, writes and closes a
same-directory temporary file, reloads it to verify the checksum and contract,
then atomically renames it. Load parses and validates into temporary values,
checks runtime compatibility, stages a new grid, and installs it with one swap.

## Startup contract

`LoadExistingMap + ConfiguredPose` loads the map before stationary startup.
The estimator and pose buffer start a new odom frame at identity. The configured
pose is startup `map_T_base`, so initial `map_T_odom` equals that configured
pose. The artifact does not restore an old runtime `map_T_odom` and does not
fabricate Keyframe history. Relocalization remains unimplemented and fail-closed.
