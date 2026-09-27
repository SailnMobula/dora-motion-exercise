"""Simulated STS3215 servos: each joint runs to its own goal at the servo's speed, independently.

The model is a velocity and acceleration limited follower per joint, as the servo's
internal position loop behaves with its acceleration register set. There is no
coordination between joints: a far goal arrives later than a near one.
"""

from __future__ import annotations

import numpy as np

from ..limits import JointLimit
from ..types import JointState


class SimServoDriver:
    name = "sim"

    def __init__(self, joints: tuple[str, ...], start: JointState, servo: JointLimit, limits: dict[str, tuple[float, float]]) -> None:
        self.joints = joints
        self.servo = servo
        self.limits = limits
        self.position = np.array([start.get(name, 0.0) for name in joints])
        self.velocity = np.zeros(len(joints))
        self.goal = self.position.copy()
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def read_joints(self) -> JointState:
        return dict(zip(self.joints, self.position.tolist()))

    def read_velocities(self) -> JointState:
        return dict(zip(self.joints, self.velocity.tolist()))

    def write_joints(self, goal: JointState) -> None:
        for index, name in enumerate(self.joints):
            if name in goal:
                lower, upper = self.limits[name]
                self.goal[index] = float(np.clip(goal[name], lower, upper))

    def step(self, dt: float) -> None:
        remaining = self.goal - self.position
        stopping_speed = np.sqrt(2.0 * self.servo.acceleration * np.abs(remaining))
        wanted = np.sign(remaining) * np.minimum(self.servo.velocity, stopping_speed)
        change = np.clip(wanted - self.velocity, -self.servo.acceleration * dt, self.servo.acceleration * dt)
        self.velocity = self.velocity + change
        moved = self.position + self.velocity * dt
        overshoot = np.sign(self.goal - moved) != np.sign(remaining)
        self.position = np.where(overshoot, self.goal, moved)
        self.velocity = np.where(overshoot, 0.0, self.velocity)
