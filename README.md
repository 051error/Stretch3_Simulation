# Stretch 3 · Simulation Environment

![Python](https://img.shields.io/badge/Python-3.12-blue?style=flat-square)
![MuJoCo](https://img.shields.io/badge/MuJoCo-Physics-green?style=flat-square)
![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-orange?style=flat-square)
![SLAM](https://img.shields.io/badge/Nav-SLAM-purple?style=flat-square)
![Platform](https://img.shields.io/badge/Platform-Linux-lightgrey?style=flat-square)

A full-stack robotics simulation environment for the Hello Robot Stretch 3 platform. Built to support sim-to-real transfer of pick-and-place manipulation tasks — integrating SLAM-based autonomous navigation, inverse kinematics, and a voice-commanded macro action system. Trained policies were successfully deployed on a physical Stretch 3 robot.

Built around the [Macro MARL PPO](https://github.com/wwlin1198/macro_marl_ppo) model developed at Northeastern University.

<br/>
<img width="817" height="344" alt="Stretch 3 simulation environment screenshot" src="https://github.com/user-attachments/assets/2fd96ebf-b1a9-40e4-9745-c2c348ec10df" />
<br/>

---

## Demos

| Demo | Description |
|------|-------------|
| [![Single pick and place](https://img.youtube.com/vi/GQZBjWnBhXU/mqdefault.jpg)](https://youtu.be/GQZBjWnBhXU) | **Single pick and place** — navigates to a table, detects an object, picks and places it |
| [![Multi-object with speech recognition](https://img.youtube.com/vi/lpIhNOAiv7I/mqdefault.jpg)](https://youtu.be/lpIhNOAiv7I) | **Multi-object + speech recognition** — full pipeline with voice-commanded macro actions |

> **Sim-to-real deployment** — source code and documentation for real-world transfer on a physical Stretch 3:
> → [Stretch3-Deployment](https://github.com/egeozgul/Stretch3-Sim-to-Real/tree/main)

<br/>
<img width="817" alt="RViz SLAM visualization" src="https://github.com/user-attachments/assets/2ba7d462-146d-4569-8743-0450cbc8b3f1" />
<br/>

*RViz visualization — map, lidar scan, and robot pose during SLAM localization*

---

## Overview

The simulation models a kitchen workspace with physical objects the robot can interact with:

- **Kitchen objects** — knife, cutting board, plates
- **Pickable ingredients** — lettuce, onion, and tomato (sized and positioned for the Stretch 3 gripper)
- **Perception** — onboard camera for object detection; lidar for navigation and mapping

The robot pipeline works as follows: lidar builds a map via SLAM → the robot navigates autonomously to named anchor points → the camera detects the target object → an IK solver drives the arm to pick and place it.

---

## Features

| | |
|---|---|
| **Physics simulation** | MuJoCo-based realistic robot dynamics |
| **ROS 2 integration** | Full communication stack with standard topics |
| **Autonomous navigation** | SLAM-based localization with anchor waypoints and turn-in-place strategy |
| **Inverse kinematics** | IK solver for precise arm positioning |
| **Speech recognition** | Voice-commanded macro actions for high-level task control |
| **Interactive control** | CLI with tab completion and command history |
| **Action system** | YAML-defined micro and macro actions, composable and RL-friendly |

---

## Quick Start

### Prerequisites

- Conda (Miniconda or Anaconda)
- Python 3.12
- ROS 2 Jazzy *(optional — required for ROS 2 features)*

### Installation

```bash
git clone <repository-url>
cd Stretch3_Simulation

conda env create -f environment_ros2.yml
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash

python verify_setup.py
```

### Running

**Terminal 1 — start the simulation**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python stretch_ros2_sim.py
```

**Terminal 2 — interactive controller**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python interactive_controller.py
```

---

## Interactive Controller

```
stretch> help                         # show all available actions
stretch> go_to_anchor anchor=A        # navigate to anchor A
stretch> elevate_arm height=0.5       # move lift to middle position
stretch> extend_arm length=0.8        # extend arm 80% of range
stretch> turn_towards anchor=ORIGIN   # face the center point
stretch> close_gripper                # grasp object
```

All movement parameters are normalized to a `0–1` range, where `0.5` is the default/middle position. Supports command history (↑/↓) and tab completion.

---

## Action Reference

### Navigation

| Action | Parameters | Description |
|--------|------------|-------------|
| `go_to_anchor` | `anchor=<A-F\|ORIGIN>` `[speed=0.5]` | Navigate to a named anchor point |
| `turn_towards` | `anchor=<A-F\|ORIGIN>` `[speed=0.5]` | Rotate to face an anchor |
| `go_to_position` | `x=<0-1>` `y=<0-1>` `[direction=<0-1>]` `[speed=0.5]` | Navigate to world coordinates |

### Arm Control

| Action | Parameters | Description |
|--------|------------|-------------|
| `reset_arm` | `[speed=0.5]` | Return arm to default pose |
| `elevate_arm` | `height=<0-1>` `[speed=0.5]` | Set lift height |
| `extend_arm` | `length=<0-1>` `[speed=0.5]` | Extend or retract the arm |
| `rotate_wrist` | `angle=<0-1>` `[speed=0.5]` | Rotate wrist yaw |
| `open_gripper` | `[speed=0.5]` | Open gripper fully |
| `close_gripper` | `[speed=0.5]` | Close gripper fully |
| `set_gripper` | `width=<0-1>` `[speed=0.5]` | Set precise gripper width |

### Utility

| Action | Parameters | Description |
|--------|------------|-------------|
| `wait` | `duration=<seconds>` | Pause for a fixed duration |
| `wait_for_arm` | `[timeout=<seconds>]` | Block until arm reaches target |

---

## Anchors

Predefined navigation waypoints in the world coordinate frame:

```
  A ——— B ——— C
  |           |
  D ——— E ——— F
        ★ ORIGIN (centroid of all anchors)
```

---

## ROS 2 Interface

### Subscribed topics

| Topic | Type | Description |
|-------|------|-------------|
| `/stretch/cmd_vel` | `Twist` | Base velocity commands |
| `/stretch/joint_commands` | `JointState` | Joint position targets |
| `/stretch/navigate_to_anchor` | `String` | Anchor navigation goal |
| `/stretch/turn_towards_anchor` | `String` | Anchor turn goal |
| `/stretch/navigate_to_position` | `Pose2D` | World-frame navigation goal |
| `/stretch/reset_arm` | `Empty` | Reset arm command |

### Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `/stretch/joint_states` | `JointState` | Current joint positions |
| `/stretch/navigation_active` | `Bool` | Navigation status flag |
| `/stretch/camera/image_raw` | `Image` | Live camera feed |

---

## Design Principles

**Normalized parameters** — all movement values use a `0–1` range (`0` = minimum, `0.5` = default, `1` = maximum), making action spaces consistent and RL-friendly.

**Action composition** — macro actions are sequences of micro actions defined in `actions.yaml`. This enables high-level commands (including those issued via speech recognition) to be built from small, reusable primitives.

**Speed control** — every movement action accepts an optional `speed` parameter for fine-grained control over execution time.

**State synchronization** — joint states are continuously published to ROS 2 topics, keeping the simulation and any external subscribers in sync.

---

## Project Structure

```
Stretch3_Simulation/
├── stretch.xml                      # Robot model (MuJoCo MJCF)
├── table_world.xml                  # Kitchen world with objects
├── actions.yaml                     # Micro and macro action definitions
├── stretch_ros2_sim.py              # Main simulation + ROS 2 node
├── interactive_controller.py        # CLI controller with tab completion
├── stretch_keyboard_controller.py   # Keyboard-driven controller
├── navigation.py                    # Anchor-based navigation logic
├── ik.py                            # Inverse kinematics solver
└── assets/                          # 3D models and textures
```

---

## Documentation

- [SETUP.md](SETUP.md) — detailed environment setup instructions
- [USAGE.md](USAGE.md) — complete usage guide with examples
- [actions.yaml](actions.yaml) — full action schema and examples

---

## Resources

- [Hello Robot — Stretch 3 docs](https://docs.hello-robot.com/)
- [MuJoCo documentation](https://mujoco.readthedocs.io/)
- [ROS 2 Jazzy documentation](https://docs.ros.org/)
- [Macro MARL PPO model](https://github.com/wwlin1198/macro_marl_ppo)
