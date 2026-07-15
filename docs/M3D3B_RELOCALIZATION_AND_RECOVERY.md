# M3-D3B Relocalization and Lost Recovery

## Ownership

SparseSlamRuntimeCore remains the only owner of the live sparse map,
map_T_odom, active observation bundles, the relocalization manager, and
localization health. Exploration only requests rotation, Stop, and replanning.
Simulation ground truth is restricted to sensor generation and assertions.

## Shared scoring

Local yaw matching and x/y/yaw relocalization call the same
SparseTofLocalMatcher::score_absolute_candidate path. It owns world-to-cell
conversion, bounded ray traversal, full planar ToF extrinsics, Hit free-path
and occupied-endpoint scoring, NoReturn free-path scoring, and
Unknown/contradiction metrics. Relocalization changes candidate generation and
acceptance bounds, not observation geometry or ReturnKind semantics.

## Search

Startup search samples deterministic known-Free map cells and the full
[-pi, pi) yaw interval. It retains bounded top-K modes, refines x/y/yaw, and
enforces a 15,000-candidate hard limit. Lost recovery first searches a bounded
neighborhood of the current prediction and escalates to global search when the
local mode is rejected or independently disproved.

Runner-up exclusion is in SE(2): a candidate is independent when its xy
distance or shortest yaw separation exceeds the configured exclusion. Score,
margin, information, unknown, contradiction, multimodal, and budget gates
remain fail-closed.

## Independent confirmation

Startup requires two different bundle IDs with increasing time ranges.
Recovery also requires independent confirmation. If a wrong local mode is
disproved, global search uses that independent bundle as a new provisional
mode. One bounded stronger-mode replacement is allowed, but it requires one
more independent bundle; no unconfirmed transform is applied.

## Startup

LoadExistingMap + Relocalization loads the immutable map, starts a fresh
identity odom frame, leaves map_T_odom invalid, locks map commits, and rotates
through the normal MotionPort. Confirmed success atomically initializes only
map_T_odom; relocalization bundles never update the map or create keyframes.

## Localization health and recovery

LowMargin, FlatCurve, Multimodal, BestAtSearchEdge, and low-information
rejections are ambiguity evidence and can only degrade health. LowScore,
UnknownDominated, Contradictory, and NoValidCandidate are consistency failures.
Lost requires bounded consecutive consistency failures plus configured elapsed
time or odom travel. Non-finite pose or stale canonical sensors can trigger
immediate Lost.

Lost sends Stop, clears the active goal/path, locks mapping, and runs local then
global recovery. Success replaces only map_T_odom, leaves odometry, map,
revision, and keyframes unchanged, then requests replanning. Failure remains
stopped and fail-closed.

## Remaining boundary

This stage does not implement continuous Full SE(2) local matching, pose graph,
loop closure, map merging, recovery keyframes, dynamic obstacle recovery, real
sensor adapters, or real motor control.
