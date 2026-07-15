# M3-E1 Exploration Quality

M3-E1 fixes the premature goal failures and false completion observed in the
first deterministic exploration run. The original tracker evaluated the
front ToF before deciding whether the robot needed to turn or move forward.
Seven lateral goals were therefore rejected at the same pose by a forward
obstacle that did not block the requested turn. Those goals and nearby cells
were then excluded permanently. Frontier detection also treated cells beyond
the configured planning bounds as Unknown, creating permanent boundary
frontiers.

## Safe goals and failure memory

Frontier candidates are reached by a bounded Free-cell search from each
cluster into known space. A candidate must satisfy configured clearance and
minimum travel distance, and bounded A* must prove it reachable. Selection
combines path cost, information gain, obstacle clearance, and prior failure
penalty. Completed goals use bounded spatial deduplication.

Failure memory stores a bounded goal position, reason, count, map revision,
and planning-cycle cooldown. A failed goal is temporarily skipped, can be
reconsidered after a configured revision change, and becomes unavailable
after the configured repeated-failure limit. Nearby but distinct goals are
not globally blacklisted.

## Tracking and obstacle confirmation

Waypoint progress is measured from the Core's estimated `map_T_base`.
Insufficient distance reduction over a bounded segment count or duration
stops motion and requests replanning. The remaining path is sampled against
the newest NavigationGridView whenever the map revision changes.

Only an explicit canonical front Hit can stop forward motion. Emergency
distance stops immediately. A normal-distance Hit requires consecutive
confirmation. Invalid, Unspecified, and explicit NoReturn readings do not
form an obstacle. A front Hit does not reject a lateral goal while the robot
is turning.

## Completion

Unknown adjacency outside configured exploration bounds is not a frontier.
`no_reachable_frontier` requires no active goal, a settled MotionPort, an Idle
Core, a fresh NavigationGridView, no reachable or cooled cluster, acceptable
coverage (or all remaining clusters proved unavailable), a map revision after
the first empty plan, and the configured number of consecutive confirmations.

The formal deterministic run completes 4 of 8 distinct selected goals (11
attempts), reaches 97.625% known cells inside the planning bounds, has zero
remaining reachable frontier clusters, confirms completion for 3 cycles,
records no collision, and saves checksum `17997707741237225682`.
