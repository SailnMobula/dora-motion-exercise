"""The real SO-101: six STS3215 servos on one serial bus, through feetech-servo-sdk.

4096 steps per turn, step 2048 is 0 rad.
Goals are clamped to the calibration's step range. Torque goes off on disconnect.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..robot import RobotConfig
from ..types import JointState

STEPS_PER_RAD = 4096 / (2 * math.pi)
CENTER_STEP = 2048
ACCELERATION_REGISTER = 254


def to_radians(step: int) -> float:
    return (step - CENTER_STEP) / STEPS_PER_RAD


def to_step(radians: float) -> int:
    return round(radians * STEPS_PER_RAD + CENTER_STEP)


@dataclass(frozen=True)
class Motor:
    id: int
    p_gain: int


@dataclass(frozen=True)
class RealConfig:
    port: str
    calibration: Path
    motors: dict[str, Motor]

    @staticmethod
    def load(path: Path | None = None) -> "RealConfig":
        config = RobotConfig.load(path)
        if "real" not in config.raw:
            raise ConnectionError(f"{config.path.name} has no real arm configured")
        raw = config.raw["real"]
        return RealConfig(
            os.environ.get("SO101_PORT", raw["port"]),
            Path(raw["calibration"]).expanduser(),
            {name: Motor(int(entry["id"]), int(entry["p_gain"])) for name, entry in raw["motors"].items()},
        )


class FeetechDriver:
    name = "real"

    def __init__(self, config: RealConfig) -> None:
        self.config = config
        self.bus = None
        self.ranges: dict[str, tuple[int, int]] = {}

    def connect(self) -> None:
        if not Path(self.config.port).exists():
            raise ConnectionError(f"no serial port {self.config.port}, set SO101_PORT")
        if not self.config.calibration.exists():
            raise ConnectionError(f"no calibration {self.config.calibration}, run lerobot-calibrate first")
        try:
            from .feetech.motor_bus import FeetechMotorsBus
        except ImportError as missing:
            raise ConnectionError("feetech-servo-sdk missing, uv sync --extra real") from missing
        calibration = json.loads(self.config.calibration.read_text())
        self.ranges = {name: (int(entry["range_min"]), int(entry["range_max"])) for name, entry in calibration.items()}
        bus = FeetechMotorsBus(port=self.config.port, motors={name: motor.id for name, motor in self.config.motors.items()})
        bus.connect()
        for name, motor in self.config.motors.items():
            bus.write("Return_Delay_Time", name, 0)
            bus.write("Maximum_Acceleration", name, ACCELERATION_REGISTER)
            bus.write("Acceleration", name, ACCELERATION_REGISTER)
            bus.write("P_Coefficient", name, motor.p_gain)
            bus.write("I_Coefficient", name, 0)
            bus.write("D_Coefficient", name, 0)
        bus.enable_torque()
        self.bus = bus

    def disconnect(self) -> None:
        if self.bus is not None:
            self.bus.disconnect(disable_torque=True)
            self.bus = None

    def is_connected(self) -> bool:
        return self.bus is not None

    def read_joints(self) -> JointState:
        if self.bus is None:
            raise ConnectionError("real arm not connected")
        return {name: to_radians(step) for name, step in self.bus.sync_read("Present_Position").items()}

    def write_joints(self, goal: JointState) -> None:
        if self.bus is None:
            raise ConnectionError("real arm not connected")
        steps = {}
        for name, radians in goal.items():
            if name not in self.config.motors:
                continue
            low, high = self.ranges.get(name, (0, 4095))
            steps[name] = min(max(to_step(radians), low), high)
        if steps:
            self.bus.sync_write("Goal_Position", steps)

    def step(self, dt: float) -> None:
        return None
