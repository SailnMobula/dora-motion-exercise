"""PTP and LIN profiles on ruckig, the two motion types of the Pilz planner.

Ruckig's jerk limit defaults to infinity, which makes its profiles acceleration
limited trapezoids, as Pilz PTP. A finite jerk in config/motion.yaml turns them into
S-curves. Phase synchronisation makes every joint start, cruise and stop together.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
from ruckig import InputParameter, Result, Ruckig, Synchronization
from ruckig import Trajectory as RuckigTrajectory

from .limits import Limits
from .types import Approach, CartesianPath, JointState, Pose, Trajectory

SAMPLE_RATE_HZ = 50.0


def _sample_times(duration: float) -> np.ndarray:
    steps = max(int(np.ceil(duration * SAMPLE_RATE_HZ)), 1)
    return np.linspace(0.0, duration, steps + 1)


def profile(
    start: np.ndarray, goal: np.ndarray, max_velocity: np.ndarray, max_acceleration: np.ndarray,
    max_jerk: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Sample times, positions, velocities and accelerations of one phase synchronised profile."""
    dof = len(start)
    if np.allclose(start, goal, atol=1e-9):
        return np.array([0.0]), np.array([start]), np.zeros((1, dof)), np.zeros((1, dof))
    parameters = InputParameter(dof)
    parameters.current_position = list(start)
    parameters.target_position = list(goal)
    parameters.max_velocity = list(max_velocity)
    parameters.max_acceleration = list(max_acceleration)
    if max_jerk is not None and np.all(np.isfinite(max_jerk)):
        parameters.max_jerk = list(max_jerk)
    parameters.synchronization = Synchronization.Phase
    trajectory = RuckigTrajectory(dof)
    result = Ruckig(dof).calculate(parameters, trajectory)
    if result not in (Result.Working, Result.Finished):
        raise ValueError(f"ruckig could not compute a profile: {result}")
    times = _sample_times(trajectory.duration)
    samples = [trajectory.at_time(t) for t in times]
    positions = np.array([sample[0] for sample in samples])
    velocities = np.array([sample[1] for sample in samples])
    accelerations = np.array([sample[2] for sample in samples])
    return times, positions, velocities, accelerations


def ptp(start: JointState, goal: JointState, limits: Limits) -> Trajectory:
    """Synchronised joint move: the slowest joint sets the pace, the others scale down with it."""
    names = tuple(goal)
    times, positions, velocities, accelerations = profile(
        np.array([start.get(name, 0.0) for name in names]),
        np.array([goal[name] for name in names]),
        np.array([limits.joint(name).velocity for name in names]),
        np.array([limits.joint(name).acceleration for name in names]),
        np.array([limits.joint(name).jerk for name in names]),
    )
    return Trajectory(names, times, positions, velocities, accelerations)


def lin(
    start: Pose, goal: Pose, limits: Limits, turn: tuple[Approach, Approach] | None = None, hold_orientation: bool = False
) -> CartesianPath:
    """Straight tool line, one profile on the path parameter s in [0, 1]. With turn, pitch and
    roll go from the first approach to the second along the same parameter. With
    hold_orientation, the orientation turns from start to goal by slerp."""
    a, b = np.array(start.position), np.array(goal.position)
    length = float(np.linalg.norm(b - a))
    if length < 1e-9:
        return CartesianPath(np.array([0.0]), (start,), (turn[1],) if turn else None)
    times, s, _, _ = profile(
        np.array([0.0]), np.array([1.0]), np.array([limits.tool_velocity / length]), np.array([limits.tool_acceleration / length])
    )
    first_rotation = pin.Quaternion(start.quaternion[3], *start.quaternion[:3])
    last_rotation = pin.Quaternion(goal.quaternion[3], *goal.quaternion[:3])
    poses = tuple(
        Pose(tuple(a + (b - a) * u), tuple(first_rotation.slerp(float(u), last_rotation).coeffs()) if hold_orientation else start.quaternion)
        for u in s[:, 0]
    )
    if turn is None:
        return CartesianPath(times, poses, None, hold_orientation)
    first, last = turn
    approaches = tuple(
        Approach(first.pitch + (last.pitch - first.pitch) * u, first.roll + (last.roll - first.roll) * u) for u in s[:, 0]
    )
    return CartesianPath(times, poses, approaches)
