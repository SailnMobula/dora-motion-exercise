# 5. Collision

Every plan passes the collision node before the ui sees it. `config/scenes/box.yaml` puts a
5 cm block beside the target.

## Wire

- `trajectory/plan` and `ik/plan` into `collision`
- `collision/plan` into the ui
- `collision/scene` into the ui

## Try

- plan to x 0.24, y -0.06, z 0.03
- refused: the block turns red, the ghost freezes at the contact
- plan to x 0.24, y -0.06, z 0.18 first, execute, then down to z 0.03

## Look for

- a straight joint move has no detour
- the via point is your planner
