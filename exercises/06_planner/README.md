# 6. Planner

A pipeline takes the first IK answer and stops at the first refusal. The planner owns the
loop: IK returns up to 8 postures, each gets a PTP and a collision check, the first free one
is the plan.

## Wire

- `ui/target` into `planner`
- `planner/<service>_request` into each service's `request`
- each service's `response` back into `planner/<service>_response`
- `planner/plan` into the ui

## Try

- plan to x 0.24, y -0.06, z 0.03 with no via point

## Look for

- note under the plan: posture 4 of 8, 3 refused
- the ghost gets to the target without a via point
- the same ik, trajectory and collision nodes as before, called as services
