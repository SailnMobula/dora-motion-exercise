# dora motion exercises

A motion stack for the SO-101 arm, built up in six exercises as a
[dora](https://github.com/dora-rs/dora) dataflow. Each exercise adds one node. The nodes are
implemented; the exercise is the wiring between them. All exercises run against a simulated
arm, the real arm is supported through the same driver interface.

## Stack

| Function | Implementation |
| --- | --- |
| Dataflow | dora-rs 1.0.1, one Python process per node, JSON messages in Arrow arrays |
| Kinematics, IK | [pinocchio](https://github.com/stack-of-tasks/pinocchio), damped least squares |
| Trajectories | [ruckig](https://github.com/pantor/ruckig), phase-synchronised PTP and LIN |
| Collision | [coal](https://github.com/coal-library/coal) on convex hulls of the URDF meshes |
| UI | [viser](https://github.com/nerfstudio-project/viser), in the browser |
| Robot | SO-101: 5 revolute arm joints, 1 gripper joint, Feetech STS3215 servos |

The SO-101 has five arm joints for a six-dimensional tool pose. `shoulder_pan` is the only
vertical axis, so the tool yaw follows from the position. IK solves position, pitch and roll:
five equations for five joints. With the gripper pointing straight down, the tool point
reaches 93 mm above the base at most.

## Exercises

| # | Exercise | Node added | Content |
| --- | --- | --- | --- |
| 1 | Joint space | `ui`, `driver` | joint goals straight to the servos, each joint at 3 rad/s independently |
| 2 | Time | `trajectory` | ruckig PTP: synchronised stop, trapezoid or jerk-limited S-curve |
| 3 | Task space | `ik` | tool position, pitch and roll to joint goals |
| 4 | Straight lines | LIN in `trajectory` and `ik` | timed Cartesian path, IK per sample at 50 Hz |
| 5 | Collision | `collision` | every plan checked against the table and a 5 cm block |
| 6 | Planner | `planner` | IK, ruckig and collision as services, up to 8 IK postures per target |

Exercises 1 to 5 form a pipeline: each node consumes its upstream output. Exercise 6 turns the
same nodes into services on request/response topics, orchestrated by a planner that retries
postures. Each exercise folder contains a `README.md` with the task.

## Requirements

- Linux, Ubuntu 22.04 or newer, x86_64 or ARM64; or Windows 10/11 with WSL2
- Chrome, Firefox or Edge
- 1 GB of disk space

### Windows: WSL2

PowerShell as administrator, then restart:

```powershell
wsl --install -d Ubuntu
```

All further commands run in the Ubuntu shell. The browser runs on Windows; WSL2 forwards
`localhost`.

### Tools

```bash
sudo apt update
sudo apt install -y git curl
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

[uv](https://docs.astral.sh/uv/) installs Python 3.12, the Python packages and the dora CLI.

Linux ARM64 additionally needs a C++ compiler and the Python headers, because ruckig builds
from source there:

```bash
sudo apt install -y build-essential python3.12-dev
```

## Setup

```bash
git clone https://github.com/SailnMobula/dora-motion-exercise.git
cd dora-motion-exercise
uv sync
```

`uv sync` installs about 900 MB into `.venv/`.

## Running an exercise

```bash
cd exercises/01_joint_space
uv run dora run dataflow.yml
```

UI on http://localhost:8080, stop with `Ctrl+C`. The UI port moves to 8081 when 8080 is taken;
the terminal prints the address.

Every input in an exercise's `dataflow.yml` is commented out. Wiring an input means
uncommenting it and naming the producing output as `<node>/<output>`:

```yaml
    inputs:
      state: driver/state
```

dora reads the dataflow at start, so every change needs a restart. The UI builds a control
only for outputs that another node consumes. Reference wiring: `solutions/`. All nodes wired
at once: `solutions/full/`.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `uv: command not found` | new terminal, or `source $HOME/.local/bin/env` |
| `Failed to build ruckig` | `sudo apt install -y build-essential python3.12-dev`, then `uv sync` |
| `fatal error: Python.h: No such file or directory` | `sudo apt install -y python3.12-dev`, then `uv sync` |
| Messages stall between nodes | one dataflow per machine; stop other `dora run` processes |

## Real arm

Requirements: a [LeRobot](https://github.com/huggingface/lerobot) calibration, which places
each joint's zero at the middle of its range, and access to the serial port.

```bash
uv sync --extra real
sudo usermod -aG dialout $USER
```

Log out and in once for the group change. The port is set in `config/robot.yaml` under
`real: port:`, default `/dev/ttyACM0`. The **Robot** dropdown in the UI switches between `sim`
and `real`. Exercise 1 has no dropdown: raw joint goals drive the servos at full speed.

WSL2 has no direct USB access. [usbipd-win](https://github.com/dorssel/usbipd-win) attaches
the device, from PowerShell as administrator:

```powershell
winget install usbipd
usbipd list
usbipd bind --busid <id>
usbipd attach --wsl --busid <id>
```

`uv run python tools/check_real.py` reads the servo positions without writing to the bus and
shows them against the URDF.

## Repository layout

| Path | Content |
| --- | --- |
| `so101/` | library without dora dependency: kinematics, IK, profiles, collision, drivers |
| `nodes/` | one dora node per file, thin wrappers around `so101/` |
| `config/robot.yaml` | joints, tool frame, home pose, table, real arm port |
| `config/motion.yaml` | PTP velocity, acceleration and jerk, LIN tool limits, sim servo |
| `config/scenes/` | obstacle boxes |
| `robot/` | SO-101 URDF, STL meshes, SRDF |
| `exercises/` | generated from `solutions/` by `uv run python tools/make_exercises.py` |
| `tests/` | `uv run pytest`: library tests and every solution in sim, about 30 s |

`ROBOT_CONFIG` selects a different robot YAML; `config/robot.yaml` is the default.

## License

[Apache License 2.0](LICENSE). The SO-101 model files in `robot/` come from
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), also Apache 2.0.
