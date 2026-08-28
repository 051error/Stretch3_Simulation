# Stretch 3 Simulation Environment

[English](README.md) · [简体中文](README.zh-CN.md)

[![Python 3.12](https://img.shields.io/badge/Python-3.12-blue?style=flat-square)]()
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2-green?style=flat-square)]()
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-orange?style=flat-square)]()
[![Linux](https://img.shields.io/badge/Platform-Linux-lightgrey?style=flat-square)]()

MuJoCo-based simulation stack for the [Hello Robot Stretch 3](https://hello-robot.com/stretch-3), built for mobile-manipulation research and sim-to-real transfer. The environment couples a physics simulation with a ROS 2 control interface, a Nav2 navigation stack, classical (IK / PID / MPC) and learned (PPO / SAC) arm controllers, and a composable YAML action system in a kitchen pick-and-place scenario.

**Highlights**

- **2D lidar + Nav2** — a 360-ray rangefinder publishes `sensor_msgs/LaserScan`, feeding Nav2 costmaps with DWB local planning and NavFn global planning (no AMCL / map_server needed)
- **Multiple manipulation controllers** — analytic IK, position PID, random-shooting MPC, and two-stage PPO/SAC policies, all selectable through the same macro-action interface
- **RL training harness** — a standalone Gymnasium environment (`StretchPickEnv`) trained with stable-baselines3 PPO or SAC, then deployed through ROS 2
- **Normalized `0–1` action space** for reproducible evaluation and RL

---

## System Overview

**Scene** — a kitchen workspace with three tables, a cutting board, knife, plates, and pickable ingredients (lettuce, onion, tomato), sized for the Stretch gripper.

**Control stack** — three interchangeable layers, from rule-based to learned:

| Layer | Method | Files |
|-------|--------|-------|
| Navigation | Built-in proportional controller to named anchors, **or** Nav2 (global + local planners) | `src/stretch_sim/navigation.py`, `launch/nav2_sim.launch.py` |
| Manipulation | IK solver, position PID, random-shooting MPC, two-stage PPO/SAC | `src/stretch_sim/ik.py`, `scripts/pid_arm_control.py`, `scripts/mpc_arm_control.py`, `scripts/rl_inference_arm.py` |
| Task composition | YAML micro/macro actions | `config/actions.yaml` |

---

## Quick Start

### Prerequisites

- Linux
- [Conda](https://docs.conda.io/en/latest/miniconda.html)
- Python 3.12
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) (for the ROS workflow)
- [Nav2](https://docs.nav2.org/) (optional, for `make nav2`): `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`

### Install

From the repository root:

```bash
conda env create -f environment/environment_ros2.yml
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash

make install   # pip install -e . + CLI entry points
make smoke     # quick automated checks (no ROS required)
make verify    # full environment check
```

For MuJoCo only (no ROS 2): `conda env create -f environment/environment.yml` → env name `simenv`.

RL training requires extra packages not in the conda spec:

```bash
pip install gymnasium stable-baselines3 tensorboard tqdm rich
```

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

**Terminal 3 — Nav2 navigation** (optional)

```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash
make nav2
```

Commands that need ROS 2 (`sim`, `nav2`, `controller`, `test`) exit with a clear message if ROS 2 is not sourced first.

| Command | Purpose |
|---------|---------|
| `make install` | `pip install -e .` (editable package + CLI tools) |
| `make smoke` | Quick automated tests (paths, MuJoCo model, ROS imports) |
| `make verify` | Full setup verification |
| `make sim` | MuJoCo + ROS 2 simulation node |
| `make nav2` | Start Nav2 navigation stack (run after `make sim`) |
| `make controller` | Interactive CLI |
| `make view` | MuJoCo viewer (world only) |
| `make test` | ROS 2 communication test |

CLI aliases available after `make install`: `stretch-sim`, `stretch-controller`, `stretch-verify`.

---

## Interactive Controller

```text
stretch> help
stretch> go_to_anchor anchor=A
stretch> elevate_arm height=0.5
stretch> extend_arm length=0.8
stretch> turn_towards anchor=ORIGIN
stretch> close_gripper
stretch> rl_get_tomato
```

Parameters are normalized to **`0–1`** (`0.5` = mid-range). The CLI supports tab completion and command history.

### Core actions

| Category | Actions |
|----------|---------|
| **Navigation** | `go_to_anchor`, `turn_towards`, `go_to_position` |
| **Arm** | `reset_arm`, `elevate_arm`, `extend_arm`, `rotate_wrist`, `open_gripper`, `close_gripper`, `set_gripper` |
| **Classical control** | `pid_pick`, `mpc_pick` |
| **RL control** | `rl_approach`, `rl_grasp_lift`, `rl_pick_object` (legacy) |
| **Utility** | `wait`, `wait_for_arm` |

Full schema and macro compositions: [config/actions.yaml](config/actions.yaml).

### Navigation anchors

Waypoints are defined as MuJoCo sites in `models/table_world.xml`:

| Anchor | Position (m) | Purpose |
|--------|-------------|---------|
| `A` | `-0.65, 1.1` | Table 1, left |
| `B` | ` 0.65, 1.1` | Table 1, right |
| `C` | `-0.65, 3.6` | Table 2, left |
| `D` | ` 0.65, 3.6` | Table 2, right |
| `G` | ` 0.5, 3.75` | Table 2, front (pre-pick waypoint for arm control) |
| `E` | ` 1.6, 2.9` | Table 3, upper |
| `F` | ` 1.6, 1.6` | Table 3, lower |
| `ORIGIN` | ` 0.53, 2.32` | Home / reference position |

---

## Navigation

### Built-in anchor navigation

The simulator node (`scripts/stretch_ros2_sim.py`) runs a proportional controller that drives the base toward a named anchor or a normalized `(x, y)` position. No map or planner is involved — it is a direct feedback controller on odometry.

### Nav2 stack

For path planning and obstacle avoidance, the model includes a **2D lidar** (360 rangefinders sweeping the horizontal plane at 1° resolution, published as `sensor_msgs/LaserScan` on `/scan`). `make nav2` launches:

- **global costmap** — rolling 15×15 m window at 0.05 m resolution
- **local costmap** — rolling 4×4 m window, DWB local planner
- **global planner** — NavFn
- **TF tree** — `map → odom` (static identity) `→ base_link` (from odometry) `→ laser` (from the URDF, via `robot_state_publisher`)
- **`robot_state_publisher`** — publishes `/robot_description` and the `base_link → laser` transform from [models/stretch_description.urdf](models/stretch_description.urdf)
- **RViz** — opens [config/nav2_view.rviz](config/nav2_view.rviz) showing the robot model, lidar, costmaps, and plans

There is no AMCL or `map_server`: the simulator publishes ground-truth odometry, and the identity `map → odom` transform anchors it in the global frame. Nav2's `cmd_vel` (in m/s and rad/s) is remapped to `/stretch/cmd_vel`, where the simulator converts it to motor commands using the measured base scaling (forward ≈ 0.25 m/s, turn ≈ 0.50 rad/s).

**Set a goal in RViz** — use the **"2D Goal Pose"** tool (`nav2_rviz_plugins/GoalTool`) or the **"Navigation 2"** panel to click a goal pose; the Nav2 stack receives it through the `/navigate_to_pose` action. The same goal can be sent from the command line:

```bash
# send a goal to the Nav2 stack
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Configuration: [config/nav2_params.yaml](config/nav2_params.yaml) · [launch/nav2_sim.launch.py](launch/nav2_sim.launch.py).

---

## Manipulation Controllers

Four interchangeable arm controllers reach and grasp a target object. Each is invoked as a micro action (`pid_pick`, `mpc_pick`, `rl_approach` + `rl_grasp_lift`) or as part of a macro (`PID_get_tomato`, `MPC_get_tomato`, `rl_get_tomato`).

| Controller | Method | Notes |
|-----------|--------|-------|
| **IK** | Analytic inverse kinematics (`src/stretch_sim/ik.py`) | Scripted motions, no optimization |
| **PID** | Position PID on lift / arm extension / wrist yaw | 5-stage sequence: descend → extend → grasp → lift |
| **MPC** | Random-shooting MPC (`scripts/mpc_arm_control.py`) | Samples K action sequences over horizon H against a kinematic model |
| **RL** | Two-stage PPO/SAC (`scripts/rl_inference_arm.py`) | Stage 1 approaches the object, stage 2 grasps and lifts |

The PID and MPC controllers run as ROS 2 nodes launched by the macro system via subprocess; they subscribe to `/stretch/joint_states` and publish on `/stretch/joint_command`.

---

## RL Training

A standalone MuJoCo environment (`src/stretch_sim/rl_env.py`, `StretchPickEnv`) trains the arm **without ROS 2 in the loop**, then the policy is deployed through the macro-action system.

```bash
# stage 1 — move the end-effector near the target
python scripts/train_rl_arm.py --stage approach --target tomato1 --algo ppo --timesteps 300000

# stage 2 — close the gripper and lift
python scripts/train_rl_arm.py --stage grasp --target tomato1 --algo ppo --timesteps 300000

# off-policy alternative (more sample-efficient)
python scripts/train_rl_arm.py --stage approach --target tomato1 --algo sac --timesteps 300000
```

Monitor training with TensorBoard:

```bash
tensorboard --logdir models/tensorboard
```

Checkpoints are written under `models/` (`<algo>_<stage>_<target>.zip`, e.g. `rl_approach_tomato1.zip`), with VecNormalize statistics saved alongside. Pretrained checkpoints for `tomato1` ship in the repo. Run a trained policy directly with `scripts/rl_inference_arm.py`, or let `rl_get_tomato` invoke both stages automatically.

---

## ROS 2 Interface

Simulation node: `scripts/stretch_ros2_sim.py`.

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/stretch/cmd_vel` | `geometry_msgs/Twist` | Base velocity |
| `/stretch/joint_command` | `std_msgs/Float64MultiArray` | Joint targets (interactive CLI) |
| `/stretch/joint_commands` | `std_msgs/Float64MultiArray` | Joint targets (keyboard / external) |
| `/stretch/navigate_to_anchor` | `std_msgs/String` | Anchor: `A`–`G` or `ORIGIN` |
| `/stretch/turn_towards_anchor` | `std_msgs/String` | Turn to face anchor |
| `/stretch/navigate_to_position` | `std_msgs/Float64MultiArray` | `[x, y, direction]` in `0–1` |
| `/stretch/reset_arm` | `std_msgs/String` | Reset arm pose |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/stretch/joint_states` | `sensor_msgs/JointState` | Current joint state |
| `/stretch/navigation_active` | `std_msgs/Bool` | `true` while navigating |
| `/stretch/camera/image_raw` | `sensor_msgs/Image` | Onboard camera (BGR8) |
| `/scan` | `sensor_msgs/LaserScan` | 2D lidar (360 rays) |
| `/odom` | `nav_msgs/Odometry` | Ground-truth odometry |

### TF tree

```
map ──(static identity)──> odom ──(dynamic from qpos)──> base_link ──(robot_state_publisher)──> laser
```

`base_link → laser` is read from the simplified URDF [models/stretch_description.urdf](models/stretch_description.urdf) and broadcast by `robot_state_publisher` (launched with `make nav2`), so the laser frame stays available in RViz and the costmaps without the simulator hard-coding it.

---

## Action System

Actions are defined in `config/actions.yaml`. **Micro actions** are primitive commands (navigate, arm motion, PID/MPC/RL control, wait); **macro actions** compose them into sequences.

| Macro | Description |
|-------|-------------|
| `get_tomato` | Scripted navigation + gripper pick |
| `rl_get_tomato` | Navigation + two-stage PPO pick |
| `PID_get_tomato` | Navigation + 5-stage PID pick |
| `MPC_get_tomato` | Navigation + shooting-MPC pick |
| `get_lettuce` / `get_onion` / `get_plate` / `go_to_knife` | Navigation to named targets (manipulation TBD) |
| `deliver` | Navigate to delivery station |

---

## Repository Layout

```text
Stretch3_Simulation/
├── Makefile                     # make sim, nav2, controller, smoke, …
├── pyproject.toml               # pip install -e . + CLI entry points
├── config/
│   ├── actions.yaml             # Micro & macro action definitions
│   ├── nav2_params.yaml         # Nav2 controller/planner/costmap parameters
│   └── nav2_view.rviz           # RViz layout (robot model, lidar, costmaps, goal tool)
├── launch/nav2_sim.launch.py    # Nav2 navigation stack (Nav2 + robot_state_publisher + RViz)
├── models/                      # MuJoCo scene + RL checkpoints + TensorBoard logs
│   └── stretch_description.urdf # Simplified robot model for RViz (robot_state_publisher)
├── src/stretch_sim/             # Library: navigation, IK, RL env, anchors, paths
├── scripts/                     # Sim node, CLI, PID/MPC/RL controllers, trainers
├── tools/                       # Model utilities (gen_laser.py, mesh tools)
├── environment/                 # Conda specs (simenv_ros2, simenv)
├── docs/                        # Setup & usage guides
├── tests/                       # ROS 2 communication tests
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
| [Nav2](https://docs.nav2.org/) | Navigation stack |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) | Middleware |
