"""Damped least squares IK as a dora node.

    target   {pose, approach, hold_orientation}   a tool position; optionally {pitch, roll},
                                      or hold_orientation for the pose's quaternion
    path     {path, start}            a LIN path, solved sample by sample
    state    {joints}                 where the robot is now, the seed
    goal ->  {joints} or {reason}     for a trajectory node
    plan ->  {trajectory, start} or {reason}    the LIN as joint trajectory

As a service for a planner:
    request {id, method: solve_many, payload: {pose, approach, hold_orientation, seed, count}}
    response -> {id, answer: {candidates} or {reason}}
"""

from __future__ import annotations

import numpy as np
from dora import Node

from so101.ik import DampedLeastSquaresIK
from so101.robot import Robot
from so101.types import Approach, CartesianPath, JointState, Pose, Trajectory
from wire import read, send


def main() -> None:
    robot = Robot()
    ik = DampedLeastSquaresIK(robot)
    current: JointState = {}
    node = Node()
    for event in node:
        if event["type"] != "INPUT":
            continue
        if event["id"] == "state":
            current = read(event)["joints"]
        elif event["id"] == "request":
            request = read(event)
            payload = request["payload"]
            candidates = ik.solve_many(
                Pose.from_dict(payload["pose"]), payload["seed"], int(payload["count"]),
                Approach.from_dict(payload.get("approach")), bool(payload.get("hold_orientation")),
            )
            answer = {"candidates": candidates} if candidates else {"reason": ik.miss_report()}
            send(node, "response", {"id": request["id"], "answer": answer})
        elif not current:
            send(node, "goal" if event["id"] == "target" else "plan", {"reason": "no robot state yet, is driver/state wired to ik/state?"})
        elif event["id"] == "target":
            message = read(event)
            solution = ik.solve(
                Pose.from_dict(message["pose"]), current, Approach.from_dict(message.get("approach")), bool(message.get("hold_orientation"))
            )
            send(node, "goal", {"joints": solution} if solution else {"reason": ik.miss_report()})
        elif event["id"] == "path":
            message = read(event)
            if "reason" in message:
                send(node, "plan", message)
                continue
            path = CartesianPath.from_dict(message["path"])
            rows, reason = ik.solve_path(path, current)
            if reason:
                send(node, "plan", {"reason": f"LIN: {reason}"})
                continue
            trajectory = Trajectory(robot.config.arm_joints, path.times, np.asarray(rows))
            send(node, "plan", {"trajectory": trajectory.to_dict(), "start": current, "note": "LIN, straight tool line"})


if __name__ == "__main__":
    main()
