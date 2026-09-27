# 2. Time

Ruckig between the sliders and the servos. Plan first, execute second.

## Wire

- `ui/motion` into `trajectory`: the Motion limits fields
- `ui/goal` into `trajectory`
- `trajectory/plan` into the ui
- `ui/execute` into the driver
- `driver/state` wherever the current joints are needed

## Try

- same move as exercise 1, press Plan to sliders
- the ghost replays the plan, the blue line is the tool path
- press Execute

## Look for

- trapezoids in the planned velocity plot
- every joint starts and stops at the same instant
- the slowest joint sets the duration, the others scale down

## Change

- Motion limits: halve the velocity, plan again
- Jerk 0, plan, then Jerk 10, plan the same move
- acceleration plot: rectangles become trapezoids, velocity corners round off
- the move takes longer: 1.17 s against 1.49 s for pan 1.0 rad, wrist -0.2 rad
