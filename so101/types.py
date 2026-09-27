"""Data that crosses module and node boundaries. Numpy and plain Python only."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

JointState = dict[str, float]


@dataclass(frozen=True)
class Pose:
    """Position in metres, orientation as a unit quaternion (x, y, z, w), in the base frame."""

    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def to_dict(self) -> dict:
        return {"position": list(self.position), "quaternion": list(self.quaternion)}

    @staticmethod
    def from_dict(data: dict) -> "Pose":
        return Pose(tuple(data["position"]), tuple(data.get("quaternion", (0.0, 0.0, 0.0, 1.0))))


@dataclass(frozen=True)
class Box:
    """An axis aligned obstacle, sizes and centre in metres. touches names robot links allowed to
    rest on it, the base on the table it stands on."""

    name: str
    size: tuple[float, float, float]
    position: tuple[float, float, float]
    touches: tuple[str, ...] = ()

    def to_dict(self) -> dict:
        return {"name": self.name, "size": list(self.size), "position": list(self.position)}

    @staticmethod
    def from_dict(data: dict) -> "Box":
        return Box(data["name"], tuple(data["size"]), tuple(data["position"]), tuple(data.get("touches", ())))


@dataclass(frozen=True)
class Approach:
    """The two orientation numbers a 5 joint arm can choose. Yaw follows from the position.

    pitch: the gripper axis below horizontal in the arm's vertical plane, rad, pi/2 is straight down
    roll: the wrist_roll joint, rad, turns the jaws about the gripper axis
    """

    pitch: float
    roll: float

    def to_dict(self) -> dict:
        return {"pitch": self.pitch, "roll": self.roll}

    @staticmethod
    def from_dict(data: dict | None) -> "Approach | None":
        return None if data is None else Approach(float(data["pitch"]), float(data["roll"]))


@dataclass(frozen=True)
class CartesianPath:
    """Timed tool poses, the output of a LIN profile and the input of path IK. With approaches,
    every sample also has a pitch and roll to hold; with hold_orientation, its full orientation."""

    times: np.ndarray
    poses: tuple[Pose, ...]
    approaches: tuple[Approach, ...] | None = None
    hold_orientation: bool = False

    def to_dict(self) -> dict:
        data = {"times": self.times.tolist(), "poses": [p.to_dict() for p in self.poses], "hold_orientation": self.hold_orientation}
        if self.approaches is not None:
            data["approaches"] = [a.to_dict() for a in self.approaches]
        return data

    @staticmethod
    def from_dict(data: dict) -> "CartesianPath":
        approaches = tuple(Approach(a["pitch"], a["roll"]) for a in data["approaches"]) if "approaches" in data else None
        return CartesianPath(
            np.asarray(data["times"], dtype=float), tuple(Pose.from_dict(p) for p in data["poses"]), approaches,
            bool(data.get("hold_orientation", False)),
        )


@dataclass(frozen=True)
class Trajectory:
    """Timed joint positions for a named set of joints."""

    joint_names: tuple[str, ...]
    times: np.ndarray
    positions: np.ndarray
    planned_velocities: np.ndarray | None = None
    planned_accelerations: np.ndarray | None = None

    @property
    def duration(self) -> float:
        return float(self.times[-1]) if len(self.times) else 0.0

    def at(self, elapsed: float) -> JointState:
        return {
            name: float(np.interp(elapsed, self.times, self.positions[:, column]))
            for column, name in enumerate(self.joint_names)
        }

    def velocities(self) -> np.ndarray:
        if self.planned_velocities is not None:
            return self.planned_velocities
        if len(self.times) < 2:
            return np.zeros_like(self.positions)
        return np.gradient(self.positions, self.times, axis=0)

    def accelerations(self) -> np.ndarray:
        if self.planned_accelerations is not None:
            return self.planned_accelerations
        if len(self.times) < 2:
            return np.zeros_like(self.positions)
        return np.gradient(self.velocities(), self.times, axis=0)

    def to_dict(self) -> dict:
        data = {"joint_names": list(self.joint_names), "times": self.times.tolist(), "positions": self.positions.tolist()}
        if self.planned_velocities is not None:
            data["velocities"] = self.planned_velocities.tolist()
        if self.planned_accelerations is not None:
            data["accelerations"] = self.planned_accelerations.tolist()
        return data

    @staticmethod
    def from_dict(data: dict) -> "Trajectory":
        velocities = np.asarray(data["velocities"], dtype=float) if "velocities" in data else None
        accelerations = np.asarray(data["accelerations"], dtype=float) if "accelerations" in data else None
        return Trajectory(
            tuple(data["joint_names"]), np.asarray(data["times"], dtype=float), np.asarray(data["positions"], dtype=float),
            velocities, accelerations,
        )


@dataclass(frozen=True)
class Collision:
    """First sample of a checked path in contact, and the link or object pairs touching there."""

    index: int
    time: float
    pairs: tuple[tuple[str, str], ...]

    def describe(self) -> str:
        contacts = ", ".join(f"{a} <-> {b}" for a, b in self.pairs[:3])
        return f"collision at {self.time:.2f} s: {contacts}"
