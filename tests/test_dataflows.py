"""Every solution dataflow in sim, with tests/probe.py in place of the ui node."""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

import pytest
import yaml

from so101.robot import Robot

ROOT = Path(__file__).resolve().parents[1]
SOLUTIONS = ROOT / "solutions"


def stop(process: subprocess.Popen) -> None:
    if process.poll() is None:
        try:
            os.killpg(process.pid, 15)
        except (ProcessLookupError, PermissionError):
            process.terminate()
    process.wait(timeout=10)


def run(exercise: str, scenario: list[dict], tmp_path: Path) -> dict:
    return run_flow(SOLUTIONS / exercise / "dataflow.yml", scenario, tmp_path)


def run_flow(source: Path, scenario: list[dict], tmp_path: Path) -> dict:
    flow = yaml.safe_load(source.read_text())
    for entry in flow["nodes"]:
        entry["path"] = str((source.parent / entry["path"]).resolve())
        if "ROBOT_CONFIG" in entry.get("env", {}):
            entry["env"]["ROBOT_CONFIG"] = str((source.parent / entry["env"]["ROBOT_CONFIG"]).resolve())
        if entry["id"] == "ui":
            entry["path"] = str(ROOT / "tests" / "probe.py")
    dataflow = tmp_path / "dataflow.yml"
    dataflow.write_text(yaml.safe_dump(flow))
    result = tmp_path / "result.json"
    env = os.environ | {"PROBE_SCENARIO": json.dumps(scenario), "PROBE_RESULT": str(result)}
    process = subprocess.Popen(
        ["dora", "run", str(dataflow), "--stop-after", "40s"], cwd=tmp_path, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 45
        while not result.exists() and time.monotonic() < deadline:
            time.sleep(0.2)
    finally:
        stop(process)
    assert result.exists(), "probe wrote no result"
    record = json.loads(result.read_text())
    assert not record.get("timeout"), record
    return record


@pytest.fixture(scope="module")
def robot() -> Robot:
    return Robot()


def test_joint_space_moves_each_servo_to_its_goal(tmp_path: Path, robot: Robot) -> None:
    goal = robot.config.home | {"shoulder_pan": 1.0, "elbow_flex": 0.3}
    record = run("01_joint_space", [{"wait": "state"}, {"send": "joints", "message": {"joints": goal}}, {"wait": "settled"}], tmp_path)
    final = record["states"][-1]["joints"]
    assert final["shoulder_pan"] == pytest.approx(1.0, abs=1e-3)
    assert final["elbow_flex"] == pytest.approx(0.3, abs=1e-3)


def test_time_plans_a_synchronised_move_and_executes_it(tmp_path: Path, robot: Robot) -> None:
    goal = robot.config.home | {"shoulder_pan": 1.0, "elbow_flex": 0.3}
    record = run("02_time", [
        {"wait": "state"}, {"send": "goal", "message": {"joints": goal}}, {"wait": "plan"},
        {"send": "execute"}, {"wait": "settled"},
    ], tmp_path)
    plan = record["plans"][0]["trajectory"]
    assert plan["velocities"][-1] == pytest.approx([0.0] * len(plan["joint_names"]), abs=1e-6)
    final = record["states"][-1]["joints"]
    assert final["shoulder_pan"] == pytest.approx(1.0, abs=0.02)


def test_task_space_reaches_the_target(tmp_path: Path, robot: Robot) -> None:
    target = [0.22, 0.08, 0.12]
    record = run("03_task_space", [
        {"wait": "state"}, {"send": "target", "message": {"pose": {"position": target}}}, {"wait": "plan"},
        {"send": "execute"}, {"wait": "settled"},
    ], tmp_path)
    tool = robot.fk(record["states"][-1]["joints"]).position
    assert tool == pytest.approx(target, abs=0.005)


def test_task_space_refuses_a_target_out_of_reach(tmp_path: Path) -> None:
    record = run("03_task_space", [
        {"wait": "state"}, {"send": "target", "message": {"pose": {"position": [0.7, 0.0, 0.1]}}}, {"wait": "plan"},
    ], tmp_path)
    assert "out of reach" in record["plans"][0]["reason"]


def test_straight_line_keeps_the_tool_on_the_line(tmp_path: Path, robot: Robot) -> None:
    start = robot.fk(robot.config.home).position
    target = [start[0] - 0.05, start[1] + 0.08, start[2] + 0.06]
    record = run("04_straight_lines", [
        {"wait": "state"}, {"send": "line", "message": {"pose": {"position": target}}}, {"wait": "plan"},
    ], tmp_path)
    plan = record["plans"][0]
    import numpy as np
    points = np.array([robot.fk(plan["start"] | dict(zip(plan["trajectory"]["joint_names"], row))).position
                       for row in plan["trajectory"]["positions"]])
    a, b = np.array(start), np.array(target)
    direction = (b - a) / np.linalg.norm(b - a)
    off_line = np.linalg.norm((points - a) - np.outer((points - a) @ direction, direction), axis=1)
    assert off_line.max() < 0.001


PICK = [0.24, -0.06, 0.03]


def test_collision_refuses_the_sweep_through_the_block(tmp_path: Path) -> None:
    record = run("05_collision", [
        {"wait": "state"}, {"send": "target", "message": {"pose": {"position": PICK}}}, {"wait": "plan"},
    ], tmp_path)
    plan = record["plans"][0]
    assert "block" in plan["reason"]
    assert "contact" in plan


def test_collision_passes_a_via_point_above_the_block(tmp_path: Path, robot: Robot) -> None:
    above = [PICK[0], PICK[1], 0.18]
    record = run("05_collision", [
        {"wait": "state"}, {"send": "target", "message": {"pose": {"position": above}}}, {"wait": "plan"},
        {"send": "execute"}, {"wait": "settled"},
        {"send": "target", "message": {"pose": {"position": PICK}}}, {"wait": "plan"},
        {"send": "execute"}, {"wait": "settled"},
    ], tmp_path)
    assert all("reason" not in plan for plan in record["plans"]), record["plans"]
    assert robot.fk(record["states"][-1]["joints"]).position == pytest.approx(PICK, abs=0.005)


def test_planner_finds_a_posture_around_the_block(tmp_path: Path, robot: Robot) -> None:
    record = run("06_planner", [
        {"wait": "state"}, {"send": "target", "message": {"pose": {"position": PICK}}}, {"wait": "plan"},
        {"send": "execute"}, {"wait": "settled"},
    ], tmp_path)
    plan = record["plans"][0]
    assert "reason" not in plan, plan
    assert "refused" in plan["note"]
    assert robot.fk(record["states"][-1]["joints"]).position == pytest.approx(PICK, abs=0.005)


def test_real_backend_without_hardware_stays_in_sim(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SO101_PORT", "/dev/does-not-exist")
    record = run("02_time", [
        {"wait": "state"}, {"send": "backend", "message": {"backend": "real"}}, {"wait": "settled"},
    ], tmp_path)
    state = record["states"][-1]
    assert state["backend"] == "sim"
    assert "no serial port" in state["error"]


def test_full_dataflow_plans_joints_target_and_line(tmp_path: Path, robot: Robot) -> None:
    start = robot.fk(robot.config.home).position
    record = run("full", [
        {"wait": "state"},
        {"send": "goal", "message": {"joints": robot.config.home | {"shoulder_pan": 0.5}}}, {"wait": "plan"},
        {"send": "target", "message": {"pose": {"position": PICK}}}, {"wait": "plan"},
        {"send": "line", "message": {"pose": {"position": [start[0], start[1], start[2] + 0.05]}}}, {"wait": "plan"},
    ], tmp_path)
    assert [("reason" in plan) for plan in record["plans"]] == [False, False, False], record["plans"]


def test_straight_line_down_holds_a_top_down_approach(tmp_path: Path, robot: Robot) -> None:
    import numpy as np
    from so101.ik import DampedLeastSquaresIK

    top_down = {"pitch": float(np.pi / 2), "roll": 0.0}
    record = run("04_straight_lines", [
        {"wait": "state"},
        {"send": "target", "message": {"pose": {"position": [0.2, 0.0, 0.07]}, "approach": top_down}}, {"wait": "plan"},
        {"send": "execute"}, {"wait": "settled"},
        {"send": "line", "message": {"pose": {"position": [0.2, 0.0, 0.01]}, "approach": top_down}}, {"wait": "plan"},
    ], tmp_path)
    plan = record["plans"][-1]
    assert "reason" not in plan, plan
    ik = DampedLeastSquaresIK(robot)
    pitches = [ik.approach_of(plan["start"] | dict(zip(plan["trajectory"]["joint_names"], row))).pitch
               for row in plan["trajectory"]["positions"]]
    assert np.degrees(np.abs(np.array(pitches) - np.pi / 2)).max() < 0.5


def test_motion_limits_from_the_ui_reach_the_plan(tmp_path: Path, robot: Robot) -> None:
    import numpy as np

    goal = robot.config.home | {"shoulder_pan": 1.0}
    record = run("02_time", [
        {"wait": "state"},
        {"send": "motion", "message": {"velocity": 0.5, "acceleration": 1.0, "jerk": None}},
        {"send": "goal", "message": {"joints": goal}}, {"wait": "plan"},
    ], tmp_path)
    velocities = np.array(record["plans"][0]["trajectory"]["velocities"])
    assert np.abs(velocities).max() == pytest.approx(0.5, abs=1e-6)



def test_collision_refuses_a_target_inside_the_table(tmp_path: Path) -> None:
    record = run("05_collision", [
        {"wait": "state"}, {"send": "target", "message": {"pose": {"position": [0.10, -0.20, -0.02]}}}, {"wait": "plan"},
    ], tmp_path)
    assert "table" in record["plans"][0]["reason"], record["plans"][0]
