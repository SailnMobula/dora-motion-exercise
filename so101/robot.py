"""The robot as a pinocchio model: arm joints, limits, the tool frame and the home pose.

ROBOT_CONFIG names the robot's YAML, config/robot.yaml (the SO-101) when unset. Paths inside it
are relative to the file. Joints outside the arm stay at their home values.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
import pinocchio as pin
import yaml

from .types import Box, JointState, Pose

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_CONFIG = ROOT / "config" / "robot.yaml"


def robot_config_path() -> Path:
    return Path(os.environ["ROBOT_CONFIG"]).resolve() if "ROBOT_CONFIG" in os.environ else DEFAULT_ROBOT_CONFIG


def home_pose(entry: dict | str, here: Path) -> JointState:
    """home as {joint: rad}, or a path to a YAML whose initial_positions are that mapping."""
    if isinstance(entry, str):
        entry = yaml.safe_load((here / entry).read_text())["initial_positions"]
    return {name: float(value) for name, value in entry.items()}


@dataclass(frozen=True)
class RobotConfig:
    """orientation: approach, pitch and roll for a 5 joint arm; pose, the full tool orientation for 6 or more."""

    path: Path
    urdf: Path
    srdf: Path
    arm_joints: tuple[str, ...]
    gripper_joint: str
    tool_frame: str
    home: JointState
    orientation: Literal["approach", "pose"]
    roll_joint: str | None
    motion: Path
    scenes: Path
    links_without_collision: tuple[str, ...]
    camera_position: tuple[float, float, float]
    camera_look_at: tuple[float, float, float]
    grid_size: float
    table: Box | None

    @staticmethod
    def load(path: Path | None = None) -> "RobotConfig":
        path = path or robot_config_path()
        raw = yaml.safe_load(path.read_text())
        here = path.parent
        view = raw["view"]
        return RobotConfig(
            path,
            (here / raw["urdf"]).resolve(),
            (here / raw["srdf"]).resolve(),
            tuple(raw["arm_joints"]),
            raw["gripper_joint"],
            raw["tool_frame"],
            home_pose(raw["home"], here),
            raw["orientation"],
            raw.get("roll_joint"),
            (here / raw["motion"]).resolve(),
            (here / raw["scenes"]).resolve(),
            tuple(raw.get("links_without_collision", ())),
            tuple(view["camera_position"]),
            tuple(view["camera_look_at"]),
            float(view["grid_size"]),
            Box.from_dict({"name": "table"} | raw["table"]) if "table" in raw else None,
        )

    @property
    def raw(self) -> dict:
        return yaml.safe_load(self.path.read_text())

    @property
    def joints(self) -> tuple[str, ...]:
        return (*self.arm_joints, self.gripper_joint)


def to_se3(pose: Pose) -> pin.SE3:
    x, y, z, w = pose.quaternion
    return pin.SE3(pin.Quaternion(w, x, y, z).normalized().matrix(), np.array(pose.position, dtype=float))


def to_pose(se3: pin.SE3) -> Pose:
    quaternion = pin.Quaternion(se3.rotation).coeffs()
    return Pose(tuple(float(v) for v in se3.translation), tuple(float(v) for v in quaternion))


class Robot:
    """Kinematic model with named joint access. Every state is {joint name: rad}; a joint a state does not name is at its home value."""

    def __init__(self, config: RobotConfig | None = None) -> None:
        self.config = config or RobotConfig.load()
        self.model = pin.buildModelFromUrdf(str(self.config.urdf))
        self.data = self.model.createData()
        self.tool_frame = self.model.getFrameId(self.config.tool_frame)
        self.arm_q = self._q_indices(self.config.arm_joints)
        self.arm_v = np.array([self.model.joints[self.model.getJointId(n)].idx_v for n in self.config.arm_joints])
        self.arm_lower = self.model.lowerPositionLimit[self.arm_q]
        self.arm_upper = self.model.upperPositionLimit[self.arm_q]
        self.home_q = pin.neutral(self.model)
        for name, value in self.config.home.items():
            self.home_q[self.model.joints[self.model.getJointId(name)].idx_q] = value

    def _q_indices(self, names: tuple[str, ...]) -> np.ndarray:
        return np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in names])

    def limits(self, name: str) -> tuple[float, float]:
        index = self.model.joints[self.model.getJointId(name)].idx_q
        return float(self.model.lowerPositionLimit[index]), float(self.model.upperPositionLimit[index])

    def configuration(self, state: JointState) -> np.ndarray:
        q = self.home_q.copy()
        for name, value in state.items():
            q[self.model.joints[self.model.getJointId(name)].idx_q] = value
        return q

    def full_state(self, state: JointState) -> JointState:
        """Every 1-DOF joint of the model, the ones state does not name at home: what a viewer draws.
        Continuous joints, wheels for instance, have two coordinates and are left out."""
        q = self.configuration(state)
        return {
            self.model.names[joint_id]: float(q[self.model.joints[joint_id].idx_q])
            for joint_id in range(1, self.model.njoints)
            if self.model.joints[joint_id].nq == 1
        }

    def named_state(self, q: np.ndarray) -> JointState:
        return {name: float(q[index]) for name, index in zip(self.config.joints, self._q_indices(self.config.joints))}

    def arm_state(self, q: np.ndarray) -> JointState:
        return {name: float(q[index]) for name, index in zip(self.config.arm_joints, self.arm_q)}

    def with_joints(self, q: np.ndarray, names: tuple[str, ...], values: np.ndarray) -> np.ndarray:
        result = q.copy()
        result[self._q_indices(names)] = values
        return result

    def tool_pose(self, q: np.ndarray) -> pin.SE3:
        pin.forwardKinematics(self.model, self.data, q)
        return pin.updateFramePlacement(self.model, self.data, self.tool_frame).copy()

    def fk(self, state: JointState) -> Pose:
        return to_pose(self.tool_pose(self.configuration(state)))
