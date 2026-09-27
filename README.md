# dora motion exercises

Build a robot motion stack for the SO-101 arm, one node at a time, in
[dora](https://github.com/dora-rs/dora) with a [viser](https://github.com/nerfstudio-project/viser)
browser UI.

Six exercises, one motion stack for the SO-101 robot arm. Every part is a ready-made node.
Your job is the wiring: which data goes into which node.

Everything runs in simulation. The real arm is optional.

## What you need

- A PC with **Linux** (Ubuntu 22.04 or newer, x86_64 or ARM64), or **Windows with WSL2**
- A browser: Chrome, Firefox or Edge
- About 1 GB of disk space

### Windows only: set up WSL2

In PowerShell, as administrator:

```powershell
wsl --install -d Ubuntu
```

Restart the PC, open **Ubuntu** from the start menu, and choose a user name and password.
Every following step runs in that Ubuntu window. The browser stays on Windows.

### Install git, curl and uv

```bash
sudo apt update
sudo apt install -y git curl
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

uv installs Python and every package for you. Nothing else to install on x86_64.

### ARM64 only: install a compiler

ruckig has no prebuilt wheel for Linux on ARM64, so `uv sync` compiles it from source. That
needs a C++ compiler and the Python 3.12 headers:

```bash
sudo apt install -y build-essential python3.12-dev
```

`python3.12-dev` is for Ubuntu 24.04, where uv uses the system Python 3.12. `uname -m` prints
`aarch64` on ARM64.

## Setup, once

```bash
git clone https://github.com/SailnMobula/dora-motion-exercise.git
cd dora-motion-exercise
uv sync
```

`uv sync` downloads Python 3.12 and about 900 MB of packages, a few minutes.

## Run an exercise

```bash
cd exercises/01_joint_space
uv run dora run dataflow.yml
```

Open **http://localhost:8080** in the browser. Stop with `Ctrl+C`.

## The task

Open `dataflow.yml` in the exercise folder. Every input is commented out:

```yaml
    inputs:
      # state:
```

Uncomment it and name where the data comes from, `<node>/<output>`:

```yaml
    inputs:
      state: driver/state
```

Restart the dataflow after every change. The browser shows a control once its output is
wired, so a half-wired dataflow still runs. Each exercise folder has a `README.md` with the
task. The solutions are in `solutions/`.

| # | Exercise | You see |
| --- | --- | --- |
| 1 | Joint space | joints arrive one after another |
| 2 | Time | all joints stop together |
| 3 | Task space | the gizmo drives the arm |
| 4 | Straight lines | a curve against a line to the same target |
| 5 | Collision | the block turns red, the plan is refused |
| 6 | Planner | several arm postures tried, the first free one wins |

Everything at once: `cd solutions/full && uv run dora run dataflow.yml`.

## Problems

| Symptom | Fix |
| --- | --- |
| `uv: command not found` | open a new terminal, or `source $HOME/.local/bin/env` |
| `Failed to build ruckig`, `Could not find the compiler` | ARM64: `sudo apt install -y build-essential python3.12-dev`, then `uv sync` |
| `fatal error: Python.h: No such file or directory` | `sudo apt install -y python3.12-dev`, then `uv sync` |
| Browser shows nothing | the terminal prints the address, e.g. `http://localhost:8081` if 8080 was taken |
| An old dataflow still runs | `Ctrl+C` in its terminal; only one dataflow at a time |

## The real arm (optional)

The arm needs a [LeRobot](https://github.com/huggingface/lerobot) calibration file first.

```bash
uv sync --extra real
sudo usermod -aG dialout $USER     # serial port access, then log out and in again
```

Set the port in `config/robot.yaml` under `real: port:`, usually `/dev/ttyACM0`. In the
browser, switch **Robot** to `real`.

On WSL2, the USB port has to be passed through from Windows first, in PowerShell as
administrator: `winget install usbipd`, then `usbipd list`, `usbipd bind --busid <id>` and
`usbipd attach --wsl --busid <id>`.

## Repository layout

| Path | Content |
| --- | --- |
| `so101/` | library: kinematics, IK, ruckig, coal collision, drivers |
| `nodes/` | one dora node per file |
| `config/` | robot, motion limits, table, scenes |
| `exercises/` | written by `uv run python tools/make_exercises.py` from `solutions/` |
| `tools/check_real.py` | read-only check of the real arm against the URDF |
| `tests/` | `uv run pytest`, about 30 s |

`ROBOT_CONFIG` points at another robot's YAML, `config/robot.yaml` is the default.

## Built with

| Part | Library |
| --- | --- |
| Dataflow | [dora-rs](https://github.com/dora-rs/dora) |
| Kinematics, IK | [pinocchio](https://github.com/stack-of-tasks/pinocchio) |
| Trajectories | [ruckig](https://github.com/pantor/ruckig) |
| Collision | [coal](https://github.com/coal-library/coal) |
| UI | [viser](https://github.com/nerfstudio-project/viser) |
| Robot model | [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), see `robot/README.md` |
