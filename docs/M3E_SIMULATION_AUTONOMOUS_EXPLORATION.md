# M3-E Simulation Autonomous Exploration

M3-E connects the existing sparse local SLAM runtime to deterministic
frontier exploration. SparseSlamRuntimeCore remains the only owner of the
estimator, map, matcher, correction state, and keyframes.

## Planning

Each planning cycle captures one immutable sparse-map snapshot and projects it
into a bounded NavigationGridView at the sparse map resolution. Free cells
are traversable. Occupied cells and their configured footprint inflation are
blocked. Unknown and Uncertain cells are blocked; only Unknown adjacency
defines a frontier.

Frontier cells are clustered deterministically. Every representative goal is
checked by bounded eight-neighbor A*, with diagonal corner cutting forbidden.
The selected reachable cluster minimizes:

path_length - information_gain_weight * cluster_cell_count

Stable cell coordinates break ties. Failed goals have bounded ID and spatial
cooldown so an evolving cluster boundary cannot cause infinite reselection.

## Motion and local SLAM

The waypoint tracker consumes only the Core's estimated map_T_base. It maps
each path segment onto the existing discrete MotionPort:

1. Stop before a motion-group transition.
2. TurnLeft or TurnRight in bounded TTL segments until odometry yaw aligns.
3. Forward in bounded TTL segments until odometry reaches the waypoint.
4. Stop and replan when the canonical front ToF reports a close Hit.

Alignment runs the existing active observation flow:

BeginCollection -> motion -> Stop -> WaitingForMotionSettle -> Matcher ->
atomic map_T_odom/map/keyframe commit.

Forward motion remains in normal Core mapping, so Wheel/IMU, timed poses, and
three-ToF occupancy updates continue. The controller never calls the plant.

## Bootstrap and completion

CreateNewMap starts with a bounded in-place mapping rotation. Planning begins
after the configured known-cell and yaw-coverage thresholds, or fails if the
bounded bootstrap duration expires without enough map evidence.

Exploration completes only after the configured number of consecutive
planning cycles has no reachable frontier while the MotionPort is settled and
the Core is Idle. Completion saves one validated sparse map artifact.

## Simulation boundary

The deterministic runner owns FakeWorld2D, SimRobotPlant, canonical sensor
generation, and collision assertions. The controller, planner, and tracker do
not include or read simulation world or plant types. Ground truth is used only
for runner travel/collision acceptance, never as an algorithm input.

This stage does not implement relocalization, lost recovery, Full SE(2)
matching, dynamic local avoidance, real sensor adapters, real motor writers,
or real robot motion.
