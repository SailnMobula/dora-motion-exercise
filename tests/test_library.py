"""The library without dora: kinematics, profiles, the sim servo, collision."""

from __future__ import annotations

import numpy as np
import pytest

from so101 import limits, profiles
from so101.collision import Box, CollisionWorld, robot_geometry
from so101.drivers.sim import SimServoDriver
from so101.ik import DampedLeastSquaresIK
from so101.robot import Robot
from so101.types import Pose


@pytest.fixture(scope="module")
def robot() -> Robot:
    return Robot()


def test_home_is_free_of_self_collision(robot: Robot) -> None:
    assert CollisionWorld(robot).check(robot.config.home) == ()


def test_ptp_joints_stop_together(robot: Robot) -> None:
    home = robot.config.home
    trajectory = profiles.ptp(home, home | {"shoulder_pan": 1.0, "wrist_flex": -0.2}, limits.load())
    moving = np.abs(trajectory.velocities()) > 1e-6
    last_moving = [np.flatnonzero(moving[:, column]).max() for column in (0, 3)]
    assert last_moving[0] == last_moving[1]


def test_sim_servos_arrive_one_after_another(robot: Robot) -> None:
    home = robot.config.home
    driver = SimServoDriver(robot.config.joints, home, limits.load().servo, {n: robot.limits(n) for n in robot.config.joints})
    driver.write_joints({"shoulder_pan": 1.5, "wrist_flex": 0.4})
    arrived: dict[str, float] = {}
    for step in range(400):
        driver.step(0.01)
        for name, goal in (("shoulder_pan", 1.5), ("wrist_flex", 0.4)):
            if name not in arrived and abs(driver.read_joints()[name] - goal) < 1e-4:
                arrived[name] = step * 0.01
    assert arrived["wrist_flex"] < arrived["shoulder_pan"] - 0.2


def test_ik_position_round_trip(robot: Robot) -> None:
    solution = DampedLeastSquaresIK(robot).solve(Pose((0.2, -0.1, 0.1)), robot.config.home)
    assert solution is not None
    assert robot.fk(robot.config.home | solution).position == pytest.approx((0.2, -0.1, 0.1), abs=1e-3)


def test_box_in_the_way_is_reported_with_the_link(robot: Robot) -> None:
    tool = robot.fk(robot.config.home).position
    world = CollisionWorld(robot, (Box("box", (0.04, 0.04, 0.04), tool),))
    pairs = world.check(robot.config.home)
    assert any("box" in pair for pair in pairs)


def test_a_reused_world_answers_like_a_fresh_one(robot: Robot) -> None:
    boxes = (Box("box", (0.05, 0.05, 0.06), (0.22, -0.14, 0.03)),)
    random = np.random.default_rng(3)
    states = [robot.config.home | dict(zip(robot.config.arm_joints, random.uniform(robot.arm_lower, robot.arm_upper))) for _ in range(600)]
    arm = robot_geometry(robot)
    reused = CollisionWorld(robot, boxes, arm)
    fresh = [bool(CollisionWorld(robot, boxes, arm).check(state)) for state in states]
    assert [bool(reused.check(state)) for state in states] == fresh


def test_feetech_step_conversion_round_trips() -> None:
    from so101.drivers.feetech_driver import to_radians, to_step

    assert to_step(0.0) == 2048
    assert to_radians(to_step(1.2345)) == pytest.approx(1.2345, abs=2 * np.pi / 4096)


def test_real_backend_without_a_port_says_why(monkeypatch: pytest.MonkeyPatch) -> None:
    from so101.drivers.feetech_driver import FeetechDriver, RealConfig

    monkeypatch.setenv("SO101_PORT", "/dev/does-not-exist")
    with pytest.raises(ConnectionError, match="no serial port"):
        FeetechDriver(RealConfig.load()).connect()


def test_ik_holds_pitch_and_roll(robot: Robot) -> None:
    from so101.types import Approach

    ik = DampedLeastSquaresIK(robot)
    wanted = Approach(np.pi / 2, 0.5)
    solution = ik.solve(Pose((0.2, -0.1, 0.05)), robot.config.home, wanted)
    assert solution is not None
    reached = ik.approach_of(robot.config.home | solution)
    assert reached.pitch == pytest.approx(wanted.pitch, abs=1e-3)
    assert reached.roll == pytest.approx(wanted.roll, abs=1e-3)
    assert robot.fk(robot.config.home | solution).position == pytest.approx((0.2, -0.1, 0.05), abs=1e-3)


def test_jerk_limit_turns_the_trapezoid_into_an_s_curve(robot: Robot) -> None:
    from dataclasses import replace

    home = robot.config.home
    goal = home | {"shoulder_pan": 1.0, "wrist_flex": -0.2}
    motion = limits.load()
    jerk = 10.0

    def with_jerk(value: float) -> limits.Limits:
        return replace(motion, joints={name: replace(limit, jerk=value) for name, limit in motion.joints.items()})

    trapezoid = profiles.ptp(home, goal, with_jerk(float("inf")))
    s_curve = profiles.ptp(home, goal, with_jerk(jerk))
    step = s_curve.times[1] - s_curve.times[0]
    assert np.abs(np.diff(trapezoid.accelerations(), axis=0)).max() > 1.0
    assert np.abs(np.diff(s_curve.accelerations(), axis=0)).max() <= jerk * step * 1.01
    assert s_curve.duration > trapezoid.duration


def test_a_joint_takes_jerk_from_default(tmp_path) -> None:
    config = tmp_path / "motion.yaml"
    config.write_text(
        "joints:\n  default: {velocity: 1.5, acceleration: 3.0, jerk: 10.0}\n  shoulder_pan: {velocity: 1.0}\n"
        "lin: {velocity: 0.1, acceleration: 0.3}\nservo: {velocity: 3.0, acceleration: 20.0}\n"
    )
    pan = limits.load(config).joint("shoulder_pan")
    assert (pan.velocity, pan.acceleration, pan.jerk) == (1.0, 3.0, 10.0)
