# Multi-ToF Mounting And Frame IDs

The canonical M3-C0 mount configuration is:

- `tof_front_frame`: yaw `0`, looking forward.
- `tof_left_frame`: yaw `+90` degrees.
- `tof_right_frame`: yaw `-90` degrees.

The assumed coordinate convention is `x` forward and `y` left, so `+90` is left and `-90` is right. If the physical robot later defines left/right opposite to this naming, only the left/right name mapping should change; the yaw facts `0/+90/-90` must not be lost.

The raw contract requires unique mount IDs and unique frame IDs before future sync or snapshot stages consume the data.
