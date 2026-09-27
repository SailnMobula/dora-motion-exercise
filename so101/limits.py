"""Motion limits from the robot config's motion file, config/motion.yaml for the SO-101."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .robot import RobotConfig


@dataclass(frozen=True)
class JointLimit:
    """rad/s, rad/s^2, rad/s^3. An infinite jerk gives a trapezoid in velocity, a finite one an S-curve."""

    velocity: float
    acceleration: float
    jerk: float = math.inf


@dataclass(frozen=True)
class Limits:
    tool_velocity: float
    tool_acceleration: float
    servo: JointLimit
    joints: dict[str, JointLimit] = field(default_factory=dict)

    def joint(self, name: str) -> JointLimit:
        return self.joints.get(name, self.joints["default"])

    def with_default(self, limit: JointLimit) -> "Limits":
        """The same limit for every joint: what the ui's Motion limits fields set."""
        return Limits(self.tool_velocity, self.tool_acceleration, self.servo, {"default": limit})

    def scaled(self, factor: float) -> "Limits":
        return Limits(
            self.tool_velocity * factor,
            self.tool_acceleration * factor,
            self.servo,
            {name: JointLimit(limit.velocity * factor, limit.acceleration * factor, limit.jerk * factor) for name, limit in self.joints.items()},
        )


def _limit(entry: dict) -> JointLimit:
    return JointLimit(float(entry["velocity"]), float(entry["acceleration"]), float(entry.get("jerk", math.inf)))


def load(path: Path | None = None) -> Limits:
    raw = yaml.safe_load((path or RobotConfig.load().motion).read_text())
    return Limits(
        tool_velocity=float(raw["lin"]["velocity"]),
        tool_acceleration=float(raw["lin"]["acceleration"]),
        servo=_limit(raw["servo"]),
        joints={name: _limit(raw["joints"]["default"] | entry) for name, entry in raw["joints"].items()},
    )
