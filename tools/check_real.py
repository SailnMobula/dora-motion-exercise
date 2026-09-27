"""Read-only check of the real SO-101 against the URDF. Nothing is written to the bus.

Pings the six servos, prints their stored settings once, then reads positions at 20 Hz and
shows the URDF in viser at those angles. Move the arm by hand: every joint on screen should
follow in the same direction, and the arm at mid-range should match the URDF's zero pose.

    uv run python tools/check_real.py [--port /dev/ttyACM0]
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import viser
from viser.extras import ViserUrdf

from so101.drivers.feetech.motor_bus import FeetechMotorsBus
from so101.drivers.feetech_driver import RealConfig, to_radians
from so101.robot import Robot

SETTINGS = ("Torque_Enable", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit")
RATE_HZ = 20.0
PORT = 8095


def settings_table(bus: FeetechMotorsBus, names: list[str]) -> str:
    rows = ["| joint | " + " | ".join(SETTINGS) + " |", "| --- |" + " --- |" * len(SETTINGS)]
    values = {register: bus.sync_read(register) for register in SETTINGS}
    for name in names:
        rows.append(f"| {name} | " + " | ".join(str(values[register][name]) for register in SETTINGS) + " |")
    return "\n".join(rows)


def position_table(steps: dict[str, int], robot: Robot) -> str:
    rows = ["| joint | step | rad | deg | URDF range, deg |", "| --- | --- | --- | --- | --- |"]
    for name, step in steps.items():
        lower, upper = robot.limits(name)
        radians = to_radians(step)
        flag = "" if lower <= radians <= upper else " outside"
        rows.append(
            f"| {name} | {step} | {radians:+.3f} | {np.degrees(radians):+.1f} | "
            f"{np.degrees(lower):+.0f} to {np.degrees(upper):+.0f}{flag} |"
        )
    return "\n".join(rows)


def main() -> None:
    config = RealConfig.load()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=config.port)
    port = parser.parse_args().port

    robot = Robot()
    motors = {name: motor.id for name, motor in config.motors.items()}
    bus = FeetechMotorsBus(port=port, motors=motors)
    try:
        bus.connect(handshake=True)
    except RuntimeError as failure:
        raise SystemExit(f"{failure}\n\nIDs 2 to 6 missing on a new arm: the servos were never set up, run lerobot-setup-motors.")

    names = list(motors)
    settings = settings_table(bus, names)
    print(settings, flush=True)

    server = viser.ViserServer(port=PORT)
    server.initial_camera.position = robot.config.camera_position
    server.initial_camera.look_at = robot.config.camera_look_at
    server.scene.add_grid("/grid", width=robot.config.grid_size, height=robot.config.grid_size, cell_size=0.05)
    urdf = ViserUrdf(server, robot.config.urdf, root_node_name="/robot")
    joint_order = urdf.get_actuated_joint_names()
    server.gui.add_markdown("**Read only.** Torque untouched, move the arm by hand.")
    server.gui.add_markdown(settings)
    live = server.gui.add_markdown("")
    print(f"viser on http://localhost:{server.get_port()}, Ctrl+C to stop", flush=True)

    try:
        while True:
            steps = bus.sync_read("Present_Position")
            joints = {name: to_radians(step) for name, step in steps.items()}
            urdf.update_cfg(np.array([joints.get(name, 0.0) for name in joint_order]))
            live.content = position_table(steps, robot)
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect(disable_torque=False)
        server.stop()


if __name__ == "__main__":
    main()
