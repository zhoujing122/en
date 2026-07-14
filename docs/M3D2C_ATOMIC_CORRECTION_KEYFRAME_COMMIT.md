# M3-D2C Atomic Correction, Keyframe, and Sparse Map Commit

M3-D2C closes the local sparse SLAM transaction after a trusted YawOnly match.
It does not add FullSE2 matching, map persistence, relocalization, navigation,
hardware access, or motion output.

## Frame contract

The accepted matcher delta remains an odom-frame yaw-only right composition:

    proposed_map_T_odom =
        predicted_map_T_odom * odom_R_delta_yaw

Only MapOdomFrameState::map_T_odom is changed. Wheel/IMU odometry,
measurement-time odom_T_base, and TimedOdomPoseBuffer are not rewritten.

## Corrected observation projection

Each Frozen Bundle route is projected independently:

    corrected_map_T_base(t) =
        proposed_map_T_odom * measurement_time_odom_T_base(t)

SparseTofObservationBuilder remains the single sensor mount-yaw application
point. Hit contributes free path plus occupied endpoint; NoReturn contributes
free path only; Invalid and Unspecified are ignored. The pre-correction map
poses stored in the Bundle are not used for final map insertion.

## Atomic transaction

Preparation copies the bounded sparse grid, applies each frame as one existing
snapshot transaction, and retains repeated evidence across frames. It also
prepares a copy-on-commit Keyframe Manager state. Preparation failure discards
these temporary states without changing the live map, frame state, revision, or
Keyframe list.

After every contract and capacity check succeeds, commit consists only of:

1. swapping the prepared sparse-grid state;
2. swapping the prepared Keyframe Manager state;
3. assigning the already validated map_T_odom;
4. incrementing map revision once;
5. consuming the Bundle and returning the phase to Idle.

The map swap checks that its captured baseline still equals the live map before
mutation. No remaining commit step can reject after that swap.

## Keyframe bounds

Each immutable Keyframe records Bundle/revision IDs, collection interval,
map transforms before/after, matcher summary, ray counts, changed-cell count,
and immutable Frozen Bundle ownership. Keyframe count and total retained rays
are bounded. Capacity exhaustion rejects the transaction without consuming an
ID or Bundle.

The current scalar ToF hardware contract still applies mount yaw at a base
origin and does not apply configured sensor x/y translation. Correct sensor
origin extrinsics remain a hardware-integration blocker.
