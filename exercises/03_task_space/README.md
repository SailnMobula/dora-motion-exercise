# 3. Task space

IK in front of ruckig. The gizmo sets a tool position, IK turns it into joints.

## Wire

- `ui/target` into `ik`
- `ik/goal` into `trajectory`
- the rest as in exercise 2

## Try

- Tool tab, drag the gizmo, release: a plan appears
- set Pitch to 90 and Roll to 45, target x 0.2, y -0.08, z 0.05: gripper straight down, jaws turned
- same pitch at z 0.15: refused
- drag the gizmo 0.5 m out from the base: refused

## Look for

- the trajectory node is unchanged, it gets joints either way
- out of reach: the refusal says how many mm the nearest posture misses by
- 6 numbers in a pose, 5 joints: yaw is the one you cannot choose
- straight down tops out at z 0.09
