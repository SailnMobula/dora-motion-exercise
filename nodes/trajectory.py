"""Ruckig as a dora node.

    goal    {joints} or {reason}      PTP from the current state, all joints synchronised
    line    {pose, approach, hold_orientation}   LIN from the current tool; pitch and roll, or
                                      the full orientation, turned along it
    state   {joints}                  where the robot is now
    motion  {velocity, acceleration, jerk}   limits for every joint, jerk null for a trapezoid
    plan -> {trajectory, start} or {reason}
    path -> {path, start} or {reason}   for an IK node to solve

As a service for a planner:
    request {id, method: ptp, payload: {start, goal}}
    response -> {id, answer: {trajectory}}
"""

from __future__ import annotations

from dora import Node

import math

from so101 import limits, profiles
from so101.ik import DampedLeastSquaresIK
from so101.robot import Robot
from so101.types import Approach, JointState, Pose
from wire import read, send


def main() -> None:
    motion = limits.load()
    robot = Robot()
    ik = DampedLeastSquaresIK(robot)
    current: JointState = {}
    node = Node()
    for event in node:
        if event["type"] != "INPUT":
            continue
        if event["id"] == "state":
            current = read(event)["joints"]
            continue
        if event["id"] == "motion":
            message = read(event)
            jerk = message.get("jerk")
            motion = motion.with_default(limits.JointLimit(
                float(message["velocity"]), float(message["acceleration"]), float(jerk) if jerk else math.inf
            ))
            continue
        if event["id"] == "request":
            request = read(event)
            payload = request["payload"]
            trajectory = profiles.ptp(payload["start"], payload["goal"], motion)
            send(node, "response", {"id": request["id"], "answer": {"trajectory": trajectory.to_dict()}})
            continue
        output = "path" if event["id"] == "line" else "plan"
        message = read(event)
        if "reason" in message:
            send(node, output, message)
        elif not current:
            send(node, output, {"reason": "no robot state yet, is driver/state wired to trajectory/state?"})
        elif event["id"] == "goal":
            send(node, "plan", {"trajectory": profiles.ptp(current, message["joints"], motion).to_dict(), "start": current})
        elif event["id"] == "line":
            goal = Approach.from_dict(message.get("approach"))
            turn = (ik.approach_of(current), goal) if goal else None
            path = profiles.lin(robot.fk(current), Pose.from_dict(message["pose"]), motion, turn, bool(message.get("hold_orientation")))
            send(node, "path", {"path": path.to_dict(), "start": current})


if __name__ == "__main__":
    main()
