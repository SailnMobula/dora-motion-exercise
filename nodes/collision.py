"""Collision checking with coal as a dora node. Passes a plan on or refuses it.

    plan        {trajectory, start} or {reason}    a PTP plan
    line_plan   the same, for a LIN
    state       {joints}                           the scene goes out once the robot is known
    plan  ->    the plan unchanged, or {reason, contact}
    scene ->    {boxes, collision}                 the obstacles and the pairs in contact

As a service for a planner:
    request {id, method: check, payload: {trajectory, start}}
    response -> {id, answer: {} or {reason, contact}}

SCENE names a file in the robot config's scenes directory, config/scenes/ for the SO-101.
"""

from __future__ import annotations

import os

from dora import Node

from so101.collision import CollisionWorld, load_scene
from so101.robot import Robot
from so101.types import Trajectory
from wire import read, send


def main() -> None:
    robot = Robot()
    boxes = load_scene(robot.config.scenes / f"{os.environ.get('SCENE', 'box')}.yaml")
    world = CollisionWorld(robot, boxes)
    scene = [box.to_dict() for box in world.boxes]
    announced = False
    node = Node()

    def check(message: dict) -> dict:
        trajectory = Trajectory.from_dict(message["trajectory"])
        hit = world.check_trajectory(trajectory, message["start"])
        if hit is None:
            send(node, "scene", {"boxes": scene, "collision": []})
            return {}
        send(node, "scene", {"boxes": scene, "collision": [list(pair) for pair in hit.pairs]})
        contact = message["start"] | dict(zip(trajectory.joint_names, trajectory.positions[hit.index].tolist()))
        return {"reason": hit.describe(), "contact": contact}

    for event in node:
        if event["type"] != "INPUT":
            continue
        message = read(event)
        if event["id"] == "state":
            if not announced:
                send(node, "scene", {"boxes": scene, "collision": list(world.check(message["joints"]))})
                announced = True
        elif event["id"] == "request":
            send(node, "response", {"id": message["id"], "answer": check(message["payload"])})
        elif "reason" in message:
            send(node, "plan", message)
        else:
            refusal = check(message)
            send(node, "plan", refusal or message)


if __name__ == "__main__":
    main()
