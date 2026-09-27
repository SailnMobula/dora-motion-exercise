"""Damped least squares IK on the arm, the method behind KDL's LMA solver.

A pose has six numbers, the arm has five joints. shoulder_pan is the only joint about the
vertical, so the gripper always points within the vertical plane through the base and the
tool: yaw follows from the position. What is left to choose is an Approach, pitch and roll.

The task rows, stacked for one Newton step:
    3  tool position, world frame
    1  pitch, the gripper axis below horizontal, measured in the arm's plane
    1  roll, the wrist_roll joint itself
Without an approach only the three position rows are used and the orientation lands where it may.

An arm with six or more joints (orientation: pose in the robot config) holds the full tool
orientation instead: 3 position rows plus 3 rotation rows of log6, the KDL LMA error.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .robot import Robot, to_se3
from .types import Approach, CartesianPath, JointState, Pose

POSITION_TOLERANCE = 1e-4
ANGLE_TOLERANCE = 1e-3
DAMPING = 1e-6
MAX_ITERATIONS = 100
REACH_MARGIN = 0.01
RESTARTS = 20
GLOBAL_RESTARTS = 20
MAX_JOINT_JUMP = 0.2
DISTINCT = 0.1
UP = np.array([0.0, 0.0, 1.0])
GRIPPER_AXIS = 2


def arm_plane(position: np.ndarray) -> np.ndarray:
    """Unit horizontal direction from the base to the tool, the arm's plane."""
    horizontal = np.array([position[0], position[1], 0.0])
    length = np.linalg.norm(horizontal)
    return horizontal / length if length > 1e-6 else np.array([1.0, 0.0, 0.0])


def pitch_of(tool: pin.SE3) -> float:
    axis = tool.rotation[:, GRIPPER_AXIS]
    return float(np.arctan2(-axis @ UP, axis @ arm_plane(tool.translation)))


class DampedLeastSquaresIK:
    def __init__(self, robot: Robot, seed: int = 0) -> None:
        self.robot = robot
        self.random = np.random.default_rng(seed)
        self.miss = np.inf
        self._reach: float | None = None
        self._shoulder = self._first_joint_origin()
        roll = robot.config.roll_joint
        self._roll_index = list(robot.config.arm_joints).index(roll) if roll else None

    def _first_joint_origin(self) -> np.ndarray:
        model, data = self.robot.model, self.robot.data
        pin.forwardKinematics(model, data, self.robot.home_q)
        return data.oMi[model.getJointId(self.robot.config.arm_joints[0])].translation.copy()

    def reach(self) -> float:
        """How far the tool gets from the first joint, measured once over random postures."""
        if self._reach is None:
            random = np.random.default_rng(1)
            q = self.robot.home_q.copy()
            farthest = 0.0
            for _ in range(2000):
                q[self.robot.arm_q] = random.uniform(self.robot.arm_lower, self.robot.arm_upper)
                farthest = max(farthest, float(np.linalg.norm(self.robot.tool_pose(q).translation - self._shoulder)))
            self._reach = farthest
        return self._reach

    def approach_of(self, state: JointState) -> Approach:
        if self.robot.config.roll_joint is None:
            raise ValueError(f"{self.robot.config.path.name} has no roll_joint, pitch and roll are for orientation: approach")
        tool = self.robot.tool_pose(self.robot.configuration(state))
        return Approach(pitch_of(tool), float(state.get(self.robot.config.roll_joint, 0.0)))

    def _descend(
        self, target: np.ndarray, q: np.ndarray, approach: Approach | None, rotation: np.ndarray | None = None
    ) -> np.ndarray | None:
        """rotation, a 3x3 tool orientation, holds the full pose; approach holds pitch and roll."""
        robot = self.robot
        q = q.copy()
        position_off = np.inf
        goal = pin.SE3(rotation, target) if rotation is not None else None
        for _ in range(MAX_ITERATIONS):
            tool = robot.tool_pose(q)
            position_error = target - tool.translation
            position_off = float(np.linalg.norm(position_error))
            if goal is not None:
                error = pin.log6(tool.actInv(goal)).vector
                if position_off < POSITION_TOLERANCE and np.linalg.norm(error[3:]) < ANGLE_TOLERANCE:
                    return q
                rows = pin.computeFrameJacobian(robot.model, robot.data, q, robot.tool_frame, pin.LOCAL)[:, robot.arm_v]
                step = rows.T @ np.linalg.solve(rows @ rows.T + DAMPING * np.eye(6), error)
                q[robot.arm_q] = np.clip(q[robot.arm_q] + step, robot.arm_lower, robot.arm_upper)
                continue
            jacobian = pin.computeFrameJacobian(robot.model, robot.data, q, robot.tool_frame, pin.LOCAL_WORLD_ALIGNED)[:, robot.arm_v]
            if approach is None:
                if position_off < POSITION_TOLERANCE:
                    return q
                rows, error = jacobian[:3], position_error
            else:
                pitch_error = approach.pitch - pitch_of(tool)
                roll_error = approach.roll - q[robot.arm_q[self._roll_index]]
                if position_off < POSITION_TOLERANCE and abs(pitch_error) < ANGLE_TOLERANCE and abs(roll_error) < ANGLE_TOLERANCE:
                    return q
                tilt_axis = np.cross(UP, arm_plane(tool.translation))
                roll_row = np.eye(len(robot.arm_q))[self._roll_index]
                rows = np.vstack([jacobian[:3], tilt_axis @ jacobian[3:], roll_row])
                error = np.concatenate([position_error, [pitch_error, roll_error]])
            step = rows.T @ np.linalg.solve(rows @ rows.T + DAMPING * np.eye(rows.shape[0]), error)
            q[robot.arm_q] = np.clip(q[robot.arm_q] + step, robot.arm_lower, robot.arm_upper)
        self.miss = min(self.miss, position_off)
        return None

    def _restart(self, seed: np.ndarray, attempt: int) -> np.ndarray:
        restart = seed.copy()
        robot = self.robot
        if attempt < RESTARTS:
            restart[robot.arm_q] = np.clip(seed[robot.arm_q] + self.random.normal(0.0, 0.5, len(robot.arm_q)), robot.arm_lower, robot.arm_upper)
        else:
            restart[robot.arm_q] = self.random.uniform(robot.arm_lower, robot.arm_upper)
        return restart

    def _beyond_reach(self, target: np.ndarray) -> float:
        return float(np.linalg.norm(target - self._shoulder)) - self.reach()

    def solve(
        self, target: Pose, seed: JointState, approach: Approach | None = None, hold_orientation: bool = False
    ) -> JointState | None:
        """From the seed first, then restarts near it, then anywhere in the joint ranges.
        hold_orientation keeps target.quaternion, for an arm with orientation: pose."""
        goal = np.array(target.position, dtype=float)
        rotation = to_se3(target).rotation if hold_orientation else None
        self.miss = np.inf
        beyond = self._beyond_reach(goal)
        if beyond > REACH_MARGIN:
            self.miss = beyond
            return None
        start = self.robot.configuration(seed)
        solution = self._descend(goal, start, approach, rotation)
        for attempt in range(RESTARTS + GLOBAL_RESTARTS):
            if solution is not None:
                break
            solution = self._descend(goal, self._restart(start, attempt), approach, rotation)
        return None if solution is None else self.robot.arm_state(solution)

    def solve_many(
        self, target: Pose, seed: JointState, count: int, approach: Approach | None = None, hold_orientation: bool = False
    ) -> list[JointState]:
        """Up to count distinct postures reaching the target, the one nearest the seed first."""
        goal = np.array(target.position, dtype=float)
        rotation = to_se3(target).rotation if hold_orientation else None
        self.miss = np.inf
        if self._beyond_reach(goal) > REACH_MARGIN:
            return []
        start = self.robot.configuration(seed)
        found: list[np.ndarray] = []
        arm = self.robot.arm_q

        def keep(solution: np.ndarray | None) -> None:
            if solution is not None and all(np.max(np.abs(solution[arm] - other[arm])) > DISTINCT for other in found):
                found.append(solution)

        keep(self._descend(goal, start, approach, rotation))
        for attempt in range(RESTARTS + GLOBAL_RESTARTS):
            if len(found) >= count:
                break
            keep(self._descend(goal, self._restart(start, attempt), approach, rotation))
        found.sort(key=lambda q: float(np.max(np.abs(q[arm] - start[arm]))))
        return [self.robot.arm_state(q) for q in found]

    def solve_path(self, path: CartesianPath, seed: JointState) -> tuple[np.ndarray, str]:
        """Arm joints per sample, each seeded by the previous one. On failure the rows tracked so far and the reason."""
        robot = self.robot
        q = robot.configuration(seed)
        rows = []
        for sample, pose in enumerate(path.poses):
            approach = path.approaches[sample] if path.approaches is not None else None
            rotation = to_se3(pose).rotation if path.hold_orientation else None
            solution = self._descend(np.array(pose.position, dtype=float), q, approach, rotation)
            if solution is None:
                return np.array(rows), f"no IK solution at {path.times[sample]:.2f} s of {path.times[-1]:.2f} s"
            jump = float(np.max(np.abs(solution[robot.arm_q] - q[robot.arm_q])))
            if jump > MAX_JOINT_JUMP:
                return np.array(rows), f"joint jump of {jump:.2f} rad at {path.times[sample]:.2f} s, the line crosses a singularity"
            q = solution
            rows.append(q[robot.arm_q].copy())
        return np.array(rows), ""

    def miss_report(self) -> str:
        if not np.isfinite(self.miss):
            return "no IK solution"
        return f"out of reach, the nearest posture leaves the tool {self.miss * 1000:.0f} mm off"
