"""The driver seam: what the rest of the stack knows about a robot."""

from __future__ import annotations

from typing import Protocol

from ..types import JointState


class Driver(Protocol):
    name: str

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def is_connected(self) -> bool: ...

    def read_joints(self) -> JointState: ...

    def write_joints(self, goal: JointState) -> None: ...

    def step(self, dt: float) -> None:
        """Advance by dt seconds. A simulation moves its servos, hardware does nothing."""
        ...
