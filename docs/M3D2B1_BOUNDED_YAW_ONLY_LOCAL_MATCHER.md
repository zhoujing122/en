# M3-D2B1 Bounded Sparse Multi-ToF Yaw-Only Local Matcher

## Input and frame contract

The matcher reads only `PreparedSparseTofLocalMatchInput`: an immutable frozen
bundle, immutable reference snapshot, and predicted `map_T_odom`. Each route is
recomputed as `candidate_map_T_odom * measurement_odom_T_base`. The existing
observation builder applies the route's `base_T_sensor` yaw exactly once.

A yaw proposal is an odom-frame right composition:

`proposed_map_T_odom = predicted_map_T_odom * odom_R_delta_yaw`.

This preserves the predicted transform's x/y and changes only its yaw relation.
The runtime does not apply the proposal in D2B1.

## Bounded search

Coarse candidates cover the closed configured yaw interval in deterministic
order. Fine candidates cover one coarse step around the coarse best and are
deduplicated against coarse candidates. Separate coarse, fine, total candidate,
scored-ray, and cells-per-ray budgets fail closed; no truncated search can be
accepted. Bundles above the ray budget use deterministic uniform sampling over
the frame-major front/left/right sequence.

## Scoring

Hit rays score every pre-endpoint cell as free and the endpoint as occupied.
NoReturn rays score only their configured free-space path. Invalid and
Unspecified routes are ignored. Confidence never changes ReturnKind; zero
confidence is ignored.

For a candidate, `raw_score` is the configured sum of free agreement,
free/occupied contradiction, and occupied endpoint agreement terms. The
comparison score is:

`normalized_score = raw_score / traversed_constraint_cell_count`.

Unknown and uncertain cells contribute zero numerator but remain in the
denominator. Minimum used rays/known cells and maximum unknown/contradiction
ratios prevent low-information candidates from winning.

## Selection and rejection

Runner-up selection excludes candidates within the configured shortest-yaw
radius around best. The matcher rejects invalid input, exhausted budgets,
unknown-dominated or contradictory candidates, no valid candidate, edge best,
low score, low margin, flat score curves, and separated near-score local peaks.
The output is only a correction proposal; map correction, keyframes, and map
commit are outside D2B1.
