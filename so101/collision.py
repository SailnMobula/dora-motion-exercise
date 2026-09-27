"""Collision checking with coal, the successor of the FCL library MoveIt uses.

The world is the robot's URDF collision geometry as convex hulls, minus the SRDF's
adjacent pairs, plus boxes from a scene file. Boxes do not collide with each other.
A hull is slightly larger than its mesh and checks 20 times faster.
"""

from __future__ import annotations

from pathlib import Path

import coal
import numpy as np
import pinocchio as pin
import yaml

from .robot import Robot
from .types import Box, Collision, JointState, Trajectory


def load_scene(path: Path) -> tuple[Box, ...]:
    raw = yaml.safe_load(path.read_text()) or {}
    return tuple(Box.from_dict(entry) for entry in raw.get("boxes", []))


def robot_geometry(robot: Robot) -> pin.GeometryModel:
    """The robot's collision hulls with every pair checked except the SRDF's disabled ones.
    Links the config lists under links_without_collision are left out."""
    geometry = pin.buildGeomFromUrdf(
        robot.model, str(robot.config.urdf), pin.GeometryType.COLLISION, package_dirs=[str(robot.config.urdf.parent)]
    )
    skipped = set(robot.config.links_without_collision)
    unused = [str(part.name) for part in geometry.geometryObjects if robot.model.frames[part.parentFrame].name in skipped]
    for name in unused:
        geometry.removeGeometryObject(name)
    for part in geometry.geometryObjects:
        if isinstance(part.geometry, coal.BVHModelBase):
            part.geometry.buildConvexRepresentation(False)
            part.geometry = part.geometry.convex
    geometry.addAllCollisionPairs()
    pin.removeCollisionPairs(robot.model, geometry, str(robot.config.srdf))
    return geometry


class CollisionWorld:
    def __init__(self, robot: Robot, boxes: tuple[Box, ...] = (), arm: pin.GeometryModel | None = None) -> None:
        """arm is robot_geometry(robot), passed in to share it between worlds; built here otherwise.
        The robot config's table, if it has one, is always part of the world."""
        self.robot = robot
        table = robot.config.table
        self.boxes = boxes + ((table,) if table is not None else ())
        geometry = (arm or robot_geometry(robot)).copy()
        robot_count = len(geometry.geometryObjects)
        for box in self.boxes:
            geometry.addGeometryObject(
                pin.GeometryObject(box.name, 0, 0, pin.SE3(np.eye(3), np.array(box.position, dtype=float)), coal.Box(*box.size))
            )
        links = [robot.model.frames[part.parentFrame].name for part in geometry.geometryObjects[:robot_count]]
        for index, box in enumerate(self.boxes, start=robot_count):
            for link, link_name in enumerate(links):
                if link_name not in box.touches:
                    geometry.addCollisionPair(pin.CollisionPair(link, index))
        self.geometry = geometry
        self.geometry_data = pin.GeometryData(geometry)
        self.owners = [self._owner(g, index, robot_count) for index, g in enumerate(geometry.geometryObjects)]

    def _owner(self, geometry: pin.GeometryObject, index: int, robot_count: int) -> str:
        return self.robot.model.frames[geometry.parentFrame].name if index < robot_count else str(geometry.name)

    def _forget_guesses(self) -> None:
        """coal warm-starts GJK from the previous query. On the hulls that misses about 1 in 150 contacts, so every check starts cold."""
        for request in self.geometry_data.collisionRequests:
            request.cached_support_func_guess = np.zeros(2, dtype=np.int32)
            request.cached_gjk_guess = np.array([1.0, 0.0, 0.0])

    def check(self, state: JointState) -> tuple[tuple[str, str], ...]:
        self._forget_guesses()
        q = self.robot.configuration(state)
        pin.computeCollisions(self.robot.model, self.robot.data, self.geometry, self.geometry_data, q, False)
        return tuple(
            sorted({(self.owners[pair.first], self.owners[pair.second])
                    for pair, result in zip(self.geometry.collisionPairs, self.geometry_data.collisionResults)
                    if result.isCollision()})
        )

    def check_trajectory(self, trajectory: Trajectory, base: JointState) -> Collision | None:
        for index, time in enumerate(trajectory.times):
            state = base | dict(zip(trajectory.joint_names, trajectory.positions[index].tolist()))
            pairs = self.check(state)
            if pairs:
                return Collision(index, float(time), pairs)
        return None
