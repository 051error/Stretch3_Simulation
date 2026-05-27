
# Stretch 3 · Simulation Environment

**MuJoCo-based simulation for the Hello Robot Stretch 3 platform**  
with full ROS 2 integration, autonomous navigation, and sim-to-real transfer

<br/>

<img width="817" height="344" alt="screenshot" src="https://github.com/user-attachments/assets/2fd96ebf-b1a9-40e4-9745-c2c348ec10df" />
<br/>

</div>

---

## Deployment

The robot uses lidar for autonomous navigation and a camera for object detection, then executes pick-and-place tasks via an IK solver.

| Demo | Description |
|------|-------------|
| [![pick and place demo](https://img.youtube.com/vi/GQZBjWnBhXU/mqdefault.jpg)](https://youtu.be/GQZBjWnBhXU) | **Single pick and place** — navigates to a table, detects an object, picks and places it |
| [![pick and place long demo + speech recognition](https://img.youtube.com/vi/5GTzTurSQr8/mqdefault.jpg)](https://youtu.be/5GTzTurSQr8) | **Multi-object + speech recognition** — full pipeline with voice-commanded macro actions |

> **Sim-to-real deployment** — source code and documentation for real-world transfer:  
> → [Stretch3-Deployment](https://github.com/egeozgul/Stretch3-Sim-to-Real/tree/main)

![RViz SLAM localization](https://github.com/egeozgul/Stretch3-Sim-to-Real/blob/main/Navigation/rviz_lidar.png?raw=true)
*RViz visualization — map, lidar scan, and robot pose during SLAM localization*

---

## Overview

This environment models a kitchen workspace for testing Reinforcement Learning algorithms. It is built around the [Macro MARL PPO](https://github.com/wwlin1198/macro_marl_ppo) model developed at Northeastern Laboratory.

**Kitchen objects** — knife, cutting board, plates  
**Pickable ingredients** — lettuce (green), onion (white), tomato (red) as spherical objects  
**Robot compatibility** — all objects are sized and positioned for the Stretch 3 gripper

---

## Features

| | |
|---|---|
| **Physics simulation** | MuJoCo-based realistic robot dynamics |
| **ROS 2 integration** | Full communication stack with standard topics |
| **Interactive control** | Command-line interface with tab completion and history |
| **Autonomous navigation** | Anchor-based navigation with turn-in-place |
| **Real-time visualization** | Live camera feed and 3D viewer |
| **Action system** | YAML-defined micro and macro actions |

---

## Quick Start

### Prerequisites

- Conda (Miniconda or Anaconda)
- Python 3.12
- ROS 2 Jazzy *(optional — required for ROS 2 features)*

### Installation

```bash
git clone <repository-url>
cd Stretch2_SimulationEnv

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

All movement parameters are **normalized to a 0–1 range**, where `0.5` is the default/middle position. Supports command history (↑/↓) and tab completion.

---

## Action Reference

### Navigation

| Action | Parameters | Description |
|--------|-----------|-------------|
| `go_to_anchor` | `anchor=<A-F\|ORIGIN>` `[speed=0.5]` | Navigate to a named anchor point |
| `turn_towards` | `anchor=<A-F\|ORIGIN>` `[speed=0.5]` | Rotate to face an anchor |
| `go_to_position` | `x=<0-1>` `y=<0-1>` `[direction=<0-1>]` `[speed=0.5]` | Navigate to world coordinates |

### Arm Control

| Action | Parameters | Description |
|--------|-----------|-------------|
| `reset_arm` | `[speed=0.5]` | Return arm to default pose |
| `elevate_arm` | `height=<0-1>` `[speed=0.5]` | Set lift height |
| `extend_arm` | `length=<0-1>` `[speed=0.5]` | Extend or retract the arm |
| `rotate_wrist` | `angle=<0-1>` `[speed=0.5]` | Rotate wrist yaw |
| `open_gripper` | `[speed=0.5]` | Open gripper fully |
| `close_gripper` | `[speed=0.5]` | Close gripper fully |
| `set_gripper` | `width=<0-1>` `[speed=0.5]` | Set precise gripper width |

### Utility

| Action | Parameters | Description |
|--------|-----------|-------------|
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

## Project Structure

```
Stretch2_SimulationEnv/
├── stretch.xml                      # Robot model (MuJoCo MJCF)
├── table_world.xml                  # Kitchen world with objects
├── actions.yaml                     # Micro and macro action definitions
├── stretch_ros2_sim.py              # Main simulation + ROS 2 node
├── interactive_controller.py        # CLI controller with tab completion
├── stretch_keyboard_controller.py   # Keyboard-driven controller
├── navigation.py                    # Anchor-based navigation logic
└── assets/                          # 3D models and textures
```

---

## Design Principles

**Normalized parameters** — all movement values use a 0–1 range (`0` = minimum, `0.5` = default, `1` = maximum), making action composition predictable and RL-friendly.

**Action composition** — macro actions are built from sequenced micro actions defined in `actions.yaml`, enabling high-level commands like those issued via speech recognition.

**Speed control** — every movement action accepts an optional `speed` parameter, allowing fine-grained control over execution time.

**State synchronization** — joint states are continuously published to ROS 2 topics, keeping the simulation and any external subscribers in sync.

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

---

<div align="center">
<sub>⚠️ This project is under active development</sub>
</div>
