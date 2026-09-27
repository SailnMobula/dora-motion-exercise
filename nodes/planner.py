"""A planner that calls IK, ruckig and collision as services and tries postures in turn.

A pipeline takes the first IK answer and stops at the first refusal. Retrying needs a
loop, so here the planner owns the loop and the other nodes answer requests:

    target      {pose, approach, hold_orientation}   from the ui
    state       {joints}
    <service>_response   {id, answer}        from ik, trajectory, collision
    <service>_request -> {id, method, payload}
    plan  ->    {trajectory, start, note} or {reason}

The flow: up to POSTURES IK solutions, nearest first; for each a PTP; the first whose
PTP is collision free is the plan.
"""

from __future__ import annotations

import itertools
from collections.abc import Generator
from dataclasses import dataclass

from dora import Node

from so101.types import JointState
from wire import read, send

POSTURES = 8


@dataclass(frozen=True)
class Call:
    service: str
    method: str
    payload: dict


Flow = Generator[Call, dict, dict]


def reach(target: dict, current: JointState) -> Flow:
    request = {"pose": target["pose"], "approach": target.get("approach"), "hold_orientation": target.get("hold_orientation", False)}
    answer = yield Call("ik", "solve_many", request | {"seed": current, "count": POSTURES})
    if "reason" in answer:
        return answer
    refusals = []
    for number, posture in enumerate(answer["candidates"], start=1):
        plan = yield Call("trajectory", "ptp", {"start": current, "goal": posture})
        checked = yield Call("collision", "check", {"trajectory": plan["trajectory"], "start": current})
        if "reason" not in checked:
            note = f"posture {number} of {len(answer['candidates'])}" + (f", {len(refusals)} refused" if refusals else "")
            return {"trajectory": plan["trajectory"], "start": current, "note": note}
        refusals.append(f"posture {number}: {checked['reason']}")
    return {"reason": "every posture collides: " + "; ".join(refusals[:3])}


def main() -> None:
    node = Node()
    current: JointState = {}
    ids = itertools.count()
    running: tuple[str, Flow] | None = None

    def advance(answer: dict | None) -> None:
        nonlocal running
        assert running is not None
        request_id, flow = running
        try:
            call = flow.send(answer) if answer is not None else next(flow)
        except StopIteration as done:
            send(node, "plan", done.value)
            running = None
            return
        running = (f"{next(ids)}", flow)
        send(node, f"{call.service}_request", {"id": running[0], "method": call.method, "payload": call.payload})

    for event in node:
        if event["type"] != "INPUT":
            continue
        message = read(event)
        if event["id"] == "state":
            current = message["joints"]
        elif event["id"] == "target":
            if not current:
                send(node, "plan", {"reason": "no robot state yet, is driver/state wired to planner/state?"})
                continue
            running = ("", reach(message, dict(current)))
            advance(None)
        elif event["id"].endswith("_response") and running is not None and message["id"] == running[0]:
            advance(message["answer"])


if __name__ == "__main__":
    main()
