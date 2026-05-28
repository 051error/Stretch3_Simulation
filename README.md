# Stretch 3 Simulation Environment

[![Python 3.12](https://img.shields.io/badge/Python-3.12-blue?style=flat-square)]()
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2-green?style=flat-square)]()
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-orange?style=flat-square)]()
[![Linux](https://img.shields.io/badge/Platform-Linux-lightgrey?style=flat-square)]()

MuJoCo-based simulation stack for the [Hello Robot Stretch 3](https://hello-robot.com/stretch-3), built for manipulation research and sim-to-real transfer. The environment couples physics simulation, a ROS 2 control interface, composable task actions, and anchor-based mobile manipulation in a kitchen pick-and-place scenario.

**Highlights**

- Deployed learned policies on a physical Stretch 3 (see [sim-to-real repo](https://github.com/egeozgul/Stretch3-Sim-to-Real))
- Integrated with [Macro MARL PPO](https://github.com/wwlin1198/macro_marl_ppo) (Northeastern University) for macro-action planning
- Normalized `0–1` action space designed for RL and reproducible evaluation

<p align="center">
  <img width="800" alt="Stretch 3 simulation — kitchen workspace" src="https://github.com/user-attachments/assets/2fd96ebf-b1a9-40e4-9745-c2c348ec10df" />
</p>

---

## What This Repository Provides

| Component | Description |
|-----------|-------------|
| **Physics simulation** | Stretch 3 + kitchen scene in MuJoCo (`models/`) |
| **ROS 2 bridge** | Topics for base velocity, joints, navigation goals, and camera |
| **Navigation** | Proportional controller to named anchors and normalized world coordinates |
| **Manipulation** | IK solver, arm PID controllers, gripper and lift control |
| **Action system** | YAML-defined micro/macro actions (`config/actions.yaml`) |
| **Interfaces** | Interactive CLI, keyboard teleop, and programmatic ROS 2 clients |

**End-to-end demos** (navigation, perception, speech, SLAM in RViz) use this simulator together with external ROS 2 nodes and the deployment stack linked below—not all of that logic lives in this repo alone.

<p align="center">
  <img width="800" alt="RViz — map, lidar, and robot pose during localization" src="https://github.com/user-attachments/assets/2ba7d462-146d-4569-8743-0450cbc8b3f1" />
  <br />
  <em>Full-system demo: SLAM and localization run on the physical robot / ROS stack; this repo supplies the sim and control interface.</em>
</p>

---

## Demos

| Demo | Description |
|------|-------------|
| [![Single pick and place](https://img.youtube.com/vi/GQZBjWnBhXU/mqdefault.jpg)](https://youtu.be/GQZBjWnBhXU) | **Single pick and place** — navigate to the table, detect an object, pick, and place |
| [![Multi-object with speech recognition](https://img.youtube.com/vi/lpIhNOAiv7I/mqdefault.jpg)](https://youtu.be/lpIhNOAiv7I) | **Multi-object + speech recognition** — full pipeline with voice-commanded macro actions |

> **Sim-to-real** — deployment on a physical Stretch 3: [Stretch3-Sim-to-Real](https://github.com/egeozgul/Stretch3-Sim-to-Real)

---

## System Overview

**Scene** — Kitchen workspace with a table, cutting board, knife, plates, and pickable ingredients (lettuce, onion, tomato), sized for the Stretch gripper.

**Typical pipeline** (across sim + deployment stack):

1. Localize and map (SLAM on hardware / ROS — see deployment repo)
2. Navigate to a named anchor (`A`–`F` or `ORIGIN`)
3. Perceive target object (camera)
4. Plan and execute pick-and-place (IK + macro actions)

**In simulation**, step 2 is handled by the built-in anchor navigator; steps 1, 3, and 4 can be exercised via ROS topics, the interactive controller, or external nodes.

---

## Quick Start

### Prerequisites

- Linux
- [Conda](https://docs.conda.io/en/latest/miniconda.html)
- Python 3.12
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) (required for the full ROS workflow)

### Install

From the repository root:

```bash
git clone https://github.com/egeozgul/Stretch3_Simulation.git
cd Stretch3_Simulation

conda env create -f environment/environment_ros2.yml
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash

pip install -e .
make smoke    # quick automated checks (recommended)
make verify   # full environment check
```

For MuJoCo only (no ROS 2): `conda env create -f environment/environment.yml` → env name `simenv`.

### Run

**Terminal 1 — simulation**

```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash
make sim
```

**Terminal 2 — interactive controller**

```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash
make controller
```

| Command | Purpose |
|---------|---------|
| `make install` | `pip install -e .` (editable package + CLI tools) |
| `make smoke` | Quick automated tests (paths, MuJoCo model, ROS imports) |
| `make verify` | Full setup verification |
| `make sim` | MuJoCo + ROS 2 simulation node |
| `make controller` | Interactive CLI |
| `make view` | MuJoCo viewer (world only) |
| `make test` | ROS 2 topic / publisher smoke test |
| `make help` | List all Makefile targets |
| `stretch-sim` / `stretch-controller` / `stretch-verify` | CLI aliases (after `pip install -e .`) |
| `./bin/start_ros2_sim.sh` | One-shot launcher with conda + ROS |

See [docs/SETUP.md](docs/SETUP.md) and [docs/USAGE.md](docs/USAGE.md) for troubleshooting and examples.

---

## Interactive Controller

```text
stretch> help
stretch> go_to_anchor anchor=A
stretch> elevate_arm height=0.5
stretch> extend_arm length=0.8
stretch> turn_towards anchor=ORIGIN
stretch> close_gripper
```

Parameters are normalized to **`0–1`** (`0.5` = mid-range). The CLI supports tab completion and command history.

### Core actions

| Category | Actions |
|----------|---------|
| **Navigation** | `go_to_anchor`, `turn_towards`, `go_to_position` |
| **Arm** | `reset_arm`, `elevate_arm`, `extend_arm`, `rotate_wrist`, `open_gripper`, `close_gripper`, `set_gripper` |
| **Utility** | `wait`, `wait_for_arm` |

Full schema and macro compositions: [config/actions.yaml](config/actions.yaml).

### Navigation anchors

Waypoints are defined as MuJoCo sites in `models/table_world.xml`:

```text
  A ——— B ——— C
  |           |
  D ——— E ——— F
        ★ ORIGIN
```

---

## ROS 2 Interface

Simulation node: `scripts/stretch_ros2_sim.py`.

Joint commands use `std_msgs/Float64MultiArray` (lift, arm extension, wrist yaw, gripper, head pan/tilt). The interactive controller publishes on `/stretch/joint_command`; the sim also listens on `/stretch/joint_commands`.

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/stretch/cmd_vel` | `geometry_msgs/Twist` | Base velocity |
| `/stretch/joint_command` | `std_msgs/Float64MultiArray` | Joint targets (interactive CLI) |
| `/stretch/joint_commands` | `std_msgs/Float64MultiArray` | Joint targets (keyboard / external) |
| `/stretch/navigate_to_anchor` | `std_msgs/String` | Anchor: `A`–`F` or `ORIGIN` |
| `/stretch/turn_towards_anchor` | `std_msgs/String` | Turn to face anchor |
| `/stretch/navigate_to_position` | `std_msgs/Float64MultiArray` | `[x, y, direction]` in `0–1` |
| `/stretch/reset_arm` | `std_msgs/String` | Reset arm pose |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/stretch/joint_states` | `sensor_msgs/JointState` | Current joint state |
| `/stretch/navigation_active` | `std_msgs/Bool` | `true` while navigating |
| `/stretch/camera/image_raw` | `sensor_msgs/Image` | Onboard camera (BGR8) |

---

## Design Notes

- **Normalized action space** — Consistent `0–1` ranges across navigation and manipulation simplify RL training and sim-to-real mapping.
- **Composable actions** — Micro actions in YAML compose into macros for high-level tasks (including voice-triggered sequences in the full demo stack).
- **Optional speed** — Movement actions accept a `speed` parameter for timing control.
- **Closed-loop state** — Joint states stream on ROS 2 for monitors, loggers, and external planners.

---

## Repository Layout

```text
Stretch3_Simulation/
├── Makefile                     # make smoke, verify, sim, controller, …
├── pyproject.toml               # pip install -e . + CLI entry points
├── config/actions.yaml          # Micro & macro action definitions
├── models/                      # MuJoCo scene (stretch.xml, table_world.xml)
├── src/stretch_sim/             # Library: navigation, IK, anchors, paths
├── scripts/                     # Sim node, CLI, smoke_test.py, verify
├── environment/                 # Conda specs (simenv_ros2, simenv)
├── docs/                        # Setup & usage guides
├── tests/                       # ROS 2 communication tests
├── bin/                         # Shell launchers
├── assets/                      # Robot meshes
├── meshes/                      # Objects & convex collision meshes
└── textures/
```

---

## Documentation & Links

| Resource | |
|----------|---|
| [Setup guide](docs/SETUP.md) | Environment and ROS 2 installation |
| [Usage guide](docs/USAGE.md) | Controllers, navigation, arm control |
| [Action definitions](config/actions.yaml) | Parameters and macro examples |
| [Hello Robot Stretch 3](https://docs.hello-robot.com/) | Hardware documentation |
| [MuJoCo](https://mujoco.readthedocs.io/) | Physics engine |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) | Middleware |
