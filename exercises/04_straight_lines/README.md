# 4. Straight lines

LIN: the tool moves on a straight line. The order flips: ruckig times the line first, IK
solves every sample after.

## Wire

- `ui/line` into `trajectory`
- `trajectory/path` into `ik`
- `ik/plan` into the ui as `line_plan`
- PTP stays as in exercise 3

## Try

- set a target, plan it as PTP, look at the blue tool path
- switch Motion to LIN, plan the same target

## Look for

- PTP: straight in joint space, a curve for the tool
- LIN: straight for the tool, joints no longer synchronised trapezoids
