"""The viser front end, the same node in every exercise. A control appears when its output is wired.

Outputs, each shown only when some node takes it as an input:
    joints     {joints}                 every slider move, straight to the servos
    goal       {joints}                 the sliders on Plan, for a trajectory node
    target     {pose, approach}         the gizmo on Plan, for an IK node; approach {pitch, roll} or absent
    line       {pose, approach}         the gizmo on Plan as a straight line (LIN)
    execute    {id, trajectory}         the last plan, on Execute
    backend    {backend}                sim or real
    motion     {velocity, acceleration, jerk}   limits for every joint, jerk null for a trapezoid

Inputs:
    state      {joints, velocities, backend, playing, error}
    plan       {trajectory, start} or {reason}
    line_plan  the same, for a LIN, from a second producer
    scene      {boxes, collision}
"""

from __future__ import annotations

import os
import queue
import threading
import time
import uuid
from collections import deque

import numpy as np
import viser
import viser.uplot as uplot
from dora import Node
from viser.extras import ViserUrdf

from so101 import limits
from so101.ik import UP, arm_plane, pitch_of
from so101.robot import Robot
from so101.types import Approach, JointState, Pose, Trajectory
from wire import read, send, wired_inputs, wired_outputs

NODE_ID = os.environ.get("NODE_ID", "ui")
PORT = int(os.environ.get("VISER_PORT", "8080"))
GHOST_COLOR = (0.27, 0.62, 1.0, 0.35)
PATH_COLOR = (40, 120, 255)
BOX_COLOR = (150, 150, 150)
TABLE_COLOR = (205, 190, 165)
TABLE_OPACITY = 1.0
GRID_LIFT = 0.0005
BOX_OPACITY = 0.8
HIT_COLOR = (230, 40, 40)
JOINT_COLORS = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#17becf")
MEASURED_SECONDS = 6.0
ACCELERATION_RANGE = (-6.0, 6.0)
JOG_STEPS = {"1 mm": 0.001, "5 mm": 0.005, "20 mm": 0.02}
ORIENTATIONS = ("pitch + roll", "free")
POSE_ORIENTATIONS = ("hold gizmo rotation", "free")
AXIS_LENGTH = 0.07
AXIS_COLOR = (230, 120, 0)
JOG_AXES = {"X-": (0, -1), "X+": (0, 1), "Y-": (1, -1), "Y+": (1, 1), "Z-": (2, -1), "Z+": (2, 1)}


class Ghost:
    """The planned motion played once on a translucent copy of the arm, which then rests at the goal."""

    def __init__(self, server: viser.ViserServer, robot: Robot) -> None:
        self.server = server
        self.robot = robot
        self.urdf = ViserUrdf(server, robot.config.urdf, root_node_name="/ghost", mesh_color_override=GHOST_COLOR)
        self.names = self.urdf.get_actuated_joint_names()
        self.plan: Trajectory | None = None
        self.played: Trajectory | None = None
        self.start: JointState = {}
        self.lock = threading.Lock()
        self.show(False)
        threading.Thread(target=self._loop, daemon=True).start()

    def show(self, visible: bool) -> None:
        self.urdf.show_visual = visible

    def set(self, plan: Trajectory | None, start: JointState) -> None:
        with self.lock:
            self.plan, self.start = plan, start
        self.show(plan is not None)
        if plan is None:
            self.server.scene.remove_by_name("/plan/tool_path")
            return
        points = np.array([self.robot.fk(start | plan.at(t)).position for t in plan.times])
        if len(points) >= 2:
            self.server.scene.add_spline_catmull_rom("/plan/tool_path", points, color=PATH_COLOR, thickness=0.003)
        else:
            self.server.scene.remove_by_name("/plan/tool_path")

    def hold(self, state: JointState) -> None:
        """Freeze the ghost in one configuration, where a refused plan made contact."""
        names = tuple(state)
        self.set(Trajectory(names, np.array([0.0]), np.array([[state[name] for name in names]])), state)

    def _draw(self, state: JointState) -> None:
        full = self.robot.full_state(state)
        self.urdf.update_cfg(np.array([full.get(name, 0.0) for name in self.names]))

    def replay(self) -> None:
        with self.lock:
            self.played = None

    def _loop(self) -> None:
        while True:
            with self.lock:
                plan, start, played = self.plan, self.start, self.played
            if plan is None or plan is played:
                time.sleep(0.05)
                continue
            for elapsed in np.arange(0.0, plan.duration + 1e-9, 0.02):
                with self.lock:
                    if self.plan is not plan:
                        break
                self._draw(start | plan.at(float(elapsed)))
                time.sleep(0.02)
            else:
                self._draw(start | plan.at(plan.duration))
                with self.lock:
                    self.played = plan


class Ui:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.outbox: queue.Queue[tuple[str, dict]] = queue.Queue()
        self.outputs = wired_outputs(node, NODE_ID)
        self.inputs = wired_inputs(node, NODE_ID)
        self.robot = Robot()
        self.joints = self.robot.config.joints
        self.state: JointState = self.robot.named_state(self.robot.home_q)
        self.plan: Trajectory | None = None
        self.measured: deque[tuple[float, list[float]]] = deque()
        self.started = time.monotonic()
        self.synced = False
        self.execution = "idle"

        self.server = viser.ViserServer(port=PORT)
        self.server.gui.configure_theme(control_width="large", show_share_button=False)
        view = self.robot.config
        self.server.initial_camera.position = view.camera_position
        self.server.initial_camera.look_at = view.camera_look_at
        table = view.table
        if table is None:
            self.server.scene.add_grid("/grid", width=view.grid_size, height=view.grid_size, cell_size=0.05)
        else:
            top = table.position[2] + table.size[2] / 2 + GRID_LIFT
            self.server.scene.add_grid(
                "/grid", width=table.size[0], height=table.size[1], cell_size=0.05, position=(table.position[0], table.position[1], top)
            )
        self.arm = ViserUrdf(self.server, self.robot.config.urdf, root_node_name="/robot")
        self.arm_names = self.arm.get_actuated_joint_names()
        self.ghost = Ghost(self.server, self.robot) if {"plan", "line_plan"} & self.inputs else None
        self.boxes: dict[str, viser.BoxHandle] = {}
        table = self.robot.config.table
        if table is not None:
            self._draw_box(table.to_dict(), hit=False)

        self.status = self.server.gui.add_markdown(self._wiring_summary())
        self._build_backend()
        self._build_motion()
        tabs = self.server.gui.add_tab_group()
        self._build_joints(tabs)
        self._build_tool(tabs)
        self._build_plan()
        self._build_measured()

    def _wiring_summary(self) -> str:
        wired = ", ".join(sorted(self.outputs)) or "nothing yet"
        return f"**ui outputs wired:** {wired}"

    def _emit(self, output: str, message: dict) -> None:
        if output in self.outputs:
            self.outbox.put((output, message))

    def _build_backend(self) -> None:
        if "backend" not in self.outputs:
            return
        dropdown = self.server.gui.add_dropdown("Robot", ("sim", "real"), initial_value="sim")
        dropdown.on_update(lambda _: self._emit("backend", {"backend": dropdown.value}))

    def _build_motion(self) -> None:
        if "motion" not in self.outputs:
            return
        default = limits.load().joint("default")
        with self.server.gui.add_folder("Motion limits"):
            self.max_velocity = self.server.gui.add_number("Velocity, rad/s", default.velocity, min=0.1, max=5.0, step=0.1)
            self.max_acceleration = self.server.gui.add_number("Acceleration, rad/s²", default.acceleration, min=0.1, max=20.0, step=0.1)
            jerk = default.jerk if np.isfinite(default.jerk) else 0.0
            self.max_jerk = self.server.gui.add_number("Jerk, rad/s³", jerk, min=0.0, max=200.0, step=1.0, hint="0 is no jerk limit, a trapezoid")
        for field in (self.max_velocity, self.max_acceleration, self.max_jerk):
            field.on_update(lambda _: self._emit("motion", {
                "velocity": float(self.max_velocity.value),
                "acceleration": float(self.max_acceleration.value),
                "jerk": float(self.max_jerk.value) or None,
            }))

    def _build_joints(self, tabs: viser.GuiTabGroupHandle) -> None:
        if not {"joints", "goal"} & self.outputs:
            return
        with tabs.add_tab("Joints"):
            self.sliders = {}
            for name in self.joints:
                lower, upper = self.robot.limits(name)
                slider = self.server.gui.add_slider(name, lower, upper, 0.01, self.state[name])
                if "joints" in self.outputs:
                    slider.on_update(lambda _: self._emit("joints", {"joints": self._slider_values()}))
                self.sliders[name] = slider
            if "goal" in self.outputs:
                self.server.gui.add_button("Plan to sliders").on_click(lambda _: self._emit("goal", {"joints": self._slider_values()}))
            self.server.gui.add_button("Sliders from robot").on_click(lambda _: self._sync_sliders())

    def _slider_values(self) -> JointState:
        return {name: float(slider.value) for name, slider in self.sliders.items()}

    def _sync_sliders(self) -> None:
        for name, slider in getattr(self, "sliders", {}).items():
            slider.value = self.state.get(name, slider.value)

    def _build_tool(self, tabs: viser.GuiTabGroupHandle) -> None:
        if not {"target", "line"} & self.outputs:
            return
        tool = self.robot.fk(self.state)
        holds_pose = self.robot.config.orientation == "pose"
        self.gizmo = self.server.scene.add_transform_controls(
            "/target", scale=0.08 if not holds_pose else 0.12, disable_rotations=not holds_pose,
            position=tool.position, wxyz=self._wxyz(tool),
        )
        self.gizmo.on_drag_end(lambda _: self._plan_to_gizmo())
        self.gizmo.on_update(lambda _: self._show_target())
        with tabs.add_tab("Tool"):
            self.target_field = self.server.gui.add_vector3("Target, m", tuple(round(v, 3) for v in tool.position), step=0.005)
            self.target_field.on_update(lambda _: self._move_gizmo_to_field())
            if holds_pose:
                self.orientation = self.server.gui.add_dropdown("Orientation", POSE_ORIENTATIONS, initial_value=POSE_ORIENTATIONS[0])
            else:
                self._build_approach()
            motions = [m for m, output in (("PTP", "target"), ("LIN", "line")) if output in self.outputs]
            self.motion = self.server.gui.add_dropdown("Motion", motions, initial_value=motions[0])
            self.step = self.server.gui.add_dropdown("Jog step", tuple(JOG_STEPS), initial_value="5 mm")
            jog = self.server.gui.add_button_group("Jog", tuple(JOG_AXES))
            jog.on_click(lambda event: self._jog(event.target.value))
            self.server.gui.add_button("Plan to gizmo").on_click(lambda _: self._plan_to_gizmo())
            self.server.gui.add_button("Gizmo to tool").on_click(lambda _: self._gizmo_to_tool())

    def _build_approach(self) -> None:
        self.orientation = self.server.gui.add_dropdown("Orientation", ORIENTATIONS, initial_value=ORIENTATIONS[0])
        approach = self._current_approach()
        self.pitch = self.server.gui.add_slider(
            "Pitch, deg", -30.0, 90.0, 1.0, round(float(np.degrees(approach.pitch))), hint="gripper axis below horizontal, 90 is straight down"
        )
        roll_joint = self.robot.config.roll_joint
        lower, upper = self.robot.limits(roll_joint)
        self.roll = self.server.gui.add_slider(
            "Roll, deg", round(float(np.degrees(lower))), round(float(np.degrees(upper))), 1.0, round(float(np.degrees(approach.roll))),
            hint=f"the {roll_joint} joint",
        )
        for control in (self.orientation, self.pitch, self.roll):
            control.on_update(lambda _: self._show_gripper_axis())

    @staticmethod
    def _wxyz(pose: Pose) -> tuple[float, float, float, float]:
        x, y, z, w = pose.quaternion
        return (w, x, y, z)

    def _current_approach(self) -> Approach:
        roll_joint = self.robot.config.roll_joint
        return Approach(pitch_of(self.robot.tool_pose(self.robot.configuration(self.state))), float(self.state.get(roll_joint, 0.0)))

    def _approach(self) -> Approach | None:
        if self.robot.config.orientation == "pose" or self.orientation.value == "free":
            return None
        return Approach(float(np.radians(self.pitch.value)), float(np.radians(self.roll.value)))

    def _show_gripper_axis(self) -> None:
        approach = self._approach()
        if approach is None:
            self.server.scene.remove_by_name("/target_axis")
            return
        position = np.array(self.gizmo.position, dtype=float)
        axis = np.cos(approach.pitch) * arm_plane(position) - np.sin(approach.pitch) * UP
        segment = np.array([[position - axis * AXIS_LENGTH, position]])
        self.server.scene.add_line_segments("/target_axis", segment, colors=AXIS_COLOR, thickness=0.004)

    def _show_target(self) -> None:
        position = tuple(round(float(v), 3) for v in self.gizmo.position)
        if tuple(self.target_field.value) != position:
            self.target_field.value = position
        self._show_gripper_axis()

    def _move_gizmo_to_field(self) -> None:
        if not np.allclose(self.gizmo.position, self.target_field.value, atol=1e-4):
            self.gizmo.position = np.array(self.target_field.value)

    def _jog(self, axis_label: str) -> None:
        axis, sign = JOG_AXES[axis_label]
        position = np.array(self.robot.fk(self.state).position)
        position[axis] += sign * JOG_STEPS[self.step.value]
        self.gizmo.position = position
        self._show_target()
        self._plan_to_gizmo()

    def _plan_to_gizmo(self) -> None:
        output = "line" if self.motion.value == "LIN" else "target"
        w, x, y, z = (float(v) for v in self.gizmo.wxyz)
        message = {"pose": Pose(tuple(float(v) for v in self.gizmo.position), (x, y, z, w)).to_dict()}
        approach = self._approach()
        if approach is not None:
            message["approach"] = approach.to_dict()
        if self.robot.config.orientation == "pose" and self.orientation.value != "free":
            message["hold_orientation"] = True
        self._emit(output, message)

    def _gizmo_to_tool(self) -> None:
        tool = self.robot.fk(self.state)
        self.gizmo.position = np.array(tool.position)
        self.gizmo.wxyz = self._wxyz(tool)
        if self.robot.config.orientation == "approach":
            approach = self._current_approach()
            self.pitch.value = round(float(np.degrees(approach.pitch)))
            self.roll.value = round(float(np.degrees(approach.roll)))
        self._show_target()

    def _build_plan(self) -> None:
        if not {"plan", "line_plan"} & self.inputs:
            return
        with self.server.gui.add_folder("Plan"):
            self.plan_status = self.server.gui.add_markdown("no plan")
            if "execute" in self.outputs:
                self.execute = self.server.gui.add_button("Execute", disabled=True)
                self.execute.on_click(lambda _: self._execute())
            self.server.gui.add_button("Replay ghost").on_click(lambda _: self.ghost.replay() if self.ghost else None)
            self.plan_plot = self._joint_plot("Planned joint velocity, rad/s", np.zeros(2), np.zeros((2, len(self.joints))))
            self.acceleration_plot = self._joint_plot(
                "Planned joint acceleration, rad/s²", np.zeros(2), np.zeros((2, len(self.joints))), ACCELERATION_RANGE
            )

    def _build_measured(self) -> None:
        if "state" not in self.inputs:
            return
        with self.server.gui.add_folder("Robot"):
            self.robot_status = self.server.gui.add_markdown("waiting for driver/state")
            self.measured_plot = self._joint_plot("Measured joint velocity, rad/s", np.zeros(2), np.zeros((2, len(self.joints))))

    def _joint_plot(
        self, title: str, times: np.ndarray, velocities: np.ndarray, value_range: tuple[float, float] = (-3.5, 3.5)
    ) -> viser.GuiUplotHandle:
        series = (uplot.Series(label="t, s"),) + tuple(
            uplot.Series(label=name, stroke=JOINT_COLORS[index % len(JOINT_COLORS)], width=2)
            for index, name in enumerate(self.joints)
        )
        return self.server.gui.add_uplot(
            data=self._plot_data(times, velocities), series=series, title=title, aspect=1.6,
            scales={"x": uplot.Scale(time=False, auto=True), "y": uplot.Scale(range=value_range)},
        )

    def _plot_data(self, times: np.ndarray, velocities: np.ndarray) -> tuple[np.ndarray, ...]:
        return (times.astype(float),) + tuple(velocities[:, column].astype(float) for column in range(velocities.shape[1]))

    def _execute(self) -> None:
        if self.plan is None:
            return
        self._emit("execute", {"id": uuid.uuid4().hex[:8], "trajectory": self.plan.to_dict()})
        self.plan = None
        self.execute.disabled = True
        if self.ghost:
            self.ghost.set(None, {})
        self.plan_status.content = "executing"
        self.execution = "sent"

    def on_state(self, message: dict) -> None:
        self.state = message["joints"]
        full = self.robot.full_state(self.state)
        self.arm.update_cfg(np.array([full.get(name, 0.0) for name in self.arm_names]))
        if not self.synced:
            self._sync_sliders()
            if hasattr(self, "gizmo"):
                self._gizmo_to_tool()
            self.synced = True
        now = time.monotonic() - self.started
        velocities = message.get("velocities") or {}
        self.measured.append((now, [velocities.get(name, 0.0) for name in self.joints]))
        while self.measured and self.measured[0][0] < now - MEASURED_SECONDS:
            self.measured.popleft()
        if self.execution == "sent" and message.get("playing"):
            self.execution = "playing"
        elif self.execution == "playing" and not message.get("playing"):
            self.execution = "idle"
            self.plan_status.content = "done, no plan"
        tool = self.robot.fk(self.state).position
        lines = [f"**{message.get('backend', '?')}**, {'moving' if message.get('playing') else 'idle'}",
                 f"tool at x {tool[0] * 1000:.0f}, y {tool[1] * 1000:.0f}, z {tool[2] * 1000:.0f} mm"]
        if message.get("error"):
            lines.append(f"⚠ {message['error']}")
        self.robot_status.content = "  \n".join(lines)

    def refresh_measured(self) -> None:
        if len(self.measured) < 2 or not hasattr(self, "measured_plot"):
            return
        times = np.array([t for t, _ in self.measured])
        self.measured_plot.data = self._plot_data(times - times[-1], np.array([v for _, v in self.measured]))

    def on_plan(self, message: dict) -> None:
        if "reason" in message:
            self.plan = None
            if hasattr(self, "execute"):
                self.execute.disabled = True
            if self.ghost and "contact" in message:
                self.ghost.hold(message["contact"])
            elif self.ghost:
                self.ghost.set(None, {})
            self.plan_status.content = f"⚠ refused: {message['reason']}"
            return
        self.plan = Trajectory.from_dict(message["trajectory"])
        start = message.get("start") or self.state
        self.plan_status.content = f"{self.plan.duration:.2f} s, {len(self.plan.times)} samples" + (
            f"  \n{message['note']}" if message.get("note") else ""
        )
        self.plan_plot.data = self._plot_data(self.plan.times, self._per_joint(self.plan.velocities()))
        self.acceleration_plot.data = self._plot_data(self.plan.times, self._per_joint(self.plan.accelerations()))
        if hasattr(self, "execute"):
            self.execute.disabled = False
        if self.ghost:
            self.ghost.set(self.plan, start)

    def _per_joint(self, values: np.ndarray) -> np.ndarray:
        """Plan columns in the ui's joint order, zero for a joint the plan does not move."""
        assert self.plan is not None
        columns = np.zeros((len(self.plan.times), len(self.joints)))
        for column, name in enumerate(self.joints):
            if name in self.plan.joint_names:
                columns[:, column] = values[:, self.plan.joint_names.index(name)]
        return columns

    def _draw_box(self, box: dict, hit: bool) -> None:
        is_table = self.robot.config.table is not None and box["name"] == self.robot.config.table.name
        color = HIT_COLOR if hit else TABLE_COLOR if is_table else BOX_COLOR
        self.boxes[box["name"]] = self.server.scene.add_box(
            f"/scene/{box['name']}", color=color, dimensions=box["size"], position=box["position"],
            opacity=TABLE_OPACITY if is_table else BOX_OPACITY,
        )

    def on_scene(self, message: dict) -> None:
        touching = {name for pair in message.get("collision", []) for name in pair}
        for box in message.get("boxes", []):
            self._draw_box(box, box["name"] in touching)

    def flush(self) -> None:
        while not self.outbox.empty():
            output, message = self.outbox.get()
            send(self.node, output, message)


def main() -> None:
    node = Node()
    ui = Ui(node)
    print(f"[ui] viser on http://localhost:{ui.server.get_port()}", flush=True)
    last_plot = 0.0
    while True:
        event = node.next(timeout=0.02)
        ui.flush()
        if time.monotonic() - last_plot > 0.1:
            ui.refresh_measured()
            last_plot = time.monotonic()
        if event is None:
            continue
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        handler = {"state": ui.on_state, "plan": ui.on_plan, "line_plan": ui.on_plan, "scene": ui.on_scene}.get(event["id"])
        if handler:
            handler(read(event))


if __name__ == "__main__":
    main()
