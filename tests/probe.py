"""Stands in for the ui node: presses the buttons of one scenario and writes what came back to PROBE_RESULT."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))

from dora import Node  # noqa: E402

from wire import read, send  # noqa: E402

SCENARIO = json.loads(os.environ["PROBE_SCENARIO"])
RESULT = Path(os.environ["PROBE_RESULT"])
TIMEOUT = 20.0


def main() -> None:
    node = Node()
    record: dict = {"plans": [], "states": []}
    steps = list(SCENARIO)
    started = time.monotonic()
    waiting_for: str | None = None
    settle_until = 0.0
    plans_consumed = 0
    while steps or waiting_for:
        if time.monotonic() - started > TIMEOUT:
            record["timeout"] = True
            break
        event = node.next(timeout=0.05)
        if event is not None and event["type"] == "INPUT":
            message = read(event)
            if event["id"] == "state":
                record["state"] = message
            elif event["id"] in ("plan", "line_plan"):
                record["plans"].append(message)
            elif event["id"] == "scene":
                record["scene"] = message
        if waiting_for == "state" and "state" in record:
            waiting_for = None
        if waiting_for == "plan" and len(record["plans"]) > plans_consumed:
            plans_consumed = len(record["plans"])
            waiting_for = None
        if waiting_for == "settled":
            state = record.get("state", {})
            if state.get("playing") or time.monotonic() < settle_until:
                continue
            speeds = [abs(v) for v in state.get("velocities", {}).values()]
            if speeds and max(speeds) > 1e-3:
                continue
            record["states"].append(state)
            waiting_for = None
        if waiting_for or not steps:
            continue
        step = steps.pop(0)
        if "wait" in step:
            waiting_for = step["wait"]
            settle_until = time.monotonic() + 0.3
        elif step.get("send") == "execute":
            plan = record["plans"][-1]
            if "trajectory" not in plan:
                record["refused"] = plan.get("reason", "")
                break
            send(node, "execute", {"id": "probe", "trajectory": plan["trajectory"]})
        else:
            send(node, step["send"], step["message"])
    RESULT.write_text(json.dumps(record))


if __name__ == "__main__":
    main()
