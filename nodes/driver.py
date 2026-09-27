"""The robot: a simulated SO-101 or the real one, behind the same driver interface.

    joints       {joints: {name: rad}}          servo goals, each joint runs there on its own
    trajectory   {id, trajectory: {...}}        played sample by sample
    backend      {backend: "sim" | "real"}      switch, refused while a trajectory plays
    state   ->   {joints, velocities, backend, playing, error}
"""

from __future__ import annotations

import time

from dora import Node

from so101 import limits
from so101.drivers.feetech_driver import FeetechDriver, RealConfig
from so101.drivers.interface import Driver
from so101.drivers.sim import SimServoDriver
from so101.robot import Robot
from so101.types import JointState, Trajectory
from wire import read, send


def velocities(before: JointState, after: JointState, dt: float) -> JointState:
    return {name: (after[name] - before.get(name, after[name])) / dt for name in after} if dt > 0 else {}


def main() -> None:
    robot = Robot()
    config = robot.config
    sim = SimServoDriver(config.joints, config.home, limits.load().servo, {name: robot.limits(name) for name in config.joints})
    sim.connect()
    driver: Driver = sim
    real: FeetechDriver | None = None
    error = ""
    playing: tuple[str, Trajectory, float] | None = None
    last_tick = time.monotonic()
    last_joints = sim.read_joints()

    def switch(wanted: str) -> str:
        nonlocal driver, real
        if playing is not None:
            return "a trajectory is playing, switch after it"
        if wanted == driver.name:
            return ""
        if wanted == "sim":
            sim.position[:] = [last_joints[name] for name in config.joints]
            sim.goal[:] = sim.position
            sim.velocity[:] = 0.0
            if real is not None:
                real.disconnect()
                real = None
            driver = sim
            return ""
        try:
            real = FeetechDriver(RealConfig.load())
            real.connect()
            real.write_joints(real.read_joints())
        except (ConnectionError, OSError) as failure:
            real = None
            return f"real arm: {failure}, staying in sim"
        driver = real
        return ""

    node = Node()
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        if event["id"] == "joints":
            playing = None
            driver.write_joints(read(event)["joints"])
        elif event["id"] == "trajectory":
            message = read(event)
            playing = (message.get("id", ""), Trajectory.from_dict(message["trajectory"]), time.monotonic())
        elif event["id"] == "backend":
            error = switch(read(event)["backend"])
        elif event["id"] == "tick":
            now = time.monotonic()
            if playing is not None:
                _, trajectory, started = playing
                elapsed = min(now - started, trajectory.duration)
                driver.write_joints(trajectory.at(elapsed))
                if elapsed >= trajectory.duration:
                    playing = None
            driver.step(now - last_tick)
            joints = driver.read_joints()
            speeds = sim.read_velocities() if driver is sim else velocities(last_joints, joints, now - last_tick)
            last_tick, last_joints = now, joints
            send(node, "state", {"joints": joints, "velocities": speeds, "backend": driver.name, "playing": playing is not None, "error": error})
    if real is not None:
        real.disconnect()


if __name__ == "__main__":
    main()
