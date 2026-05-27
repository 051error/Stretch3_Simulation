# Usage Guide

Complete guide for using the Stretch 3 Simulation Environment.

## Quick Start

### Running the Simulation

**Terminal 1: Start Simulation Node**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
make sim
# or: python scripts/stretch_ros2_sim.py
```

**Terminal 2: Interactive Controller (Recommended)**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
make controller
# or: python scripts/interactive_controller.py
```

**Terminal 2: Keyboard Controller (Alternative)**
```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python scripts/stretch_keyboard_controller.py
```

## Interactive Controller

The interactive controller provides a modern command-line interface with action-based control.

### Getting Started

```bash
make controller
# or: python scripts/interactive_controller.py
```

You'll see a welcome screen with all available actions. Use:
- `help` - Show all actions
- `help <action_name>` - Show details for specific action
- `list` - List all actions
- `exit` or `quit` - Exit controller

### Command Examples

```bash
# Navigation
stretch> go_to_anchor anchor=A speed=0.5
stretch> turn_towards anchor=ORIGIN
stretch> go_to_position x=0.5 y=0.5

# Arm Control
stretch> reset_arm speed=0.3
stretch> elevate_arm height=0.5 speed=0.5
stretch> extend_arm length=0.7
stretch> rotate_wrist angle=0.5
stretch> open_gripper

# Utility
stretch> wait duration=2.0
stretch> wait_for_arm timeout=5.0
```

### Features

- **Command History** - Use ↑/↓ arrow keys to navigate previous commands
- **Tab Completion** - Press Tab to autocomplete action names
- **Normalized Parameters** - All parameters use 0-1 range (0=min, 0.5=middle, 1=max)
- **Speed Control** - Optional speed parameter for all movements (default: 0.5)

## Keyboard Controller

Alternative control method using keyboard shortcuts.

### Controls

**Base Movement**
- `W/S` - Move forward/backward
- `A/D` - Turn left/right
- `1-6` - Navigate to anchors (A-F)

**Arm & Gripper**
- `Q/E` - Lift up/down
- `R/F` - Arm extend/retract
- `T/G` - Wrist yaw rotate
- `Z/X` - Gripper open/close

**Head**
- `Arrow Keys` - Pan/tilt head

**General**
- `0` - Reset arm
- `ESC` - Exit

## Navigation System

### Overview

The navigation system uses a proportional controller to autonomously navigate to target positions.

### How It Works

1. **Calculate Target** - Compute distance and direction to target
2. **Align Orientation** - Turn robot to face target (if needed)
3. **Move Forward** - Move forward while maintaining alignment
4. **Final Alignment** - Rotate to target direction (if specified)
5. **Arrival** - Stop when within tolerance (0.15m)

### Navigation Commands

**Via Interactive Controller**
```bash
stretch> go_to_anchor anchor=A
stretch> turn_towards anchor=ORIGIN
stretch> go_to_position x=0.5 y=0.5 speed=0.7
```

**Via ROS 2 Topics**
```bash
# Navigate to anchor
ros2 topic pub --once /stretch/navigate_to_anchor std_msgs/msg/String "{data: 'A'}"

# Turn towards anchor
ros2 topic pub --once /stretch/turn_towards_anchor std_msgs/msg/String "{data: 'ORIGIN'}"

# Navigate to position
ros2 topic pub --once /stretch/navigate_to_position std_msgs/msg/Float64MultiArray "{data: [0.5, 0.5, 50.0]}"
```

### Anchors

Predefined navigation points:
- **A, B, C, D, E, F** - Table positions
- **ORIGIN** - Center point (average of all anchors)

Anchor positions are defined in `models/table_world.xml`.

### Manual Override

Manual velocity commands automatically cancel navigation:
```bash
ros2 topic pub /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Arm Control

### Normalized Parameters

All arm control parameters use a 0-1 normalized range:
- **0.0** = Minimum value
- **0.5** = Middle/default value
- **1.0** = Maximum value

The system automatically maps these to actual joint ranges.

### Examples

```bash
# Reset arm (all joints to default)
stretch> reset_arm speed=0.3

# Elevate to middle height
stretch> elevate_arm height=0.5

# Extend arm to 75% of maximum
stretch> extend_arm length=0.75

# Rotate wrist to middle position
stretch> rotate_wrist angle=0.5

# Set gripper to half-open
stretch> set_gripper width=0.5
```

### Speed Control

All movements support optional speed control (0-1 range):
- `speed=0.0` - Very slow
- `speed=0.5` - Normal (default)
- `speed=1.0` - Maximum speed

## Testing

### Quick Test

```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
python test_ros2_communication.py
```

### Manual Testing

**1. Verify Topics**
```bash
ros2 topic list
```

**2. Test Base Movement**
```bash
ros2 topic pub --once /stretch/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

**3. Test Joint Commands**
```bash
ros2 topic pub --once /stretch/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0]}"
```

**4. Monitor Joint States**
```bash
ros2 topic echo /stretch/joint_states
```

## Troubleshooting

### Robot doesn't turn around center
- Ensure forward velocity is zero when turning
- Check navigation controller is properly initialized

### Joint commands move unexpected joints
- Joint states are automatically synced before publishing
- If issue persists, check joint state callback is receiving updates

### Navigation doesn't complete
- Check navigation status: `ros2 topic echo /stretch/navigation_active`
- Verify target was set (check logs)
- Manual commands cancel navigation

### Topics not appearing
- Ensure simulation node is running
- Verify ROS 2 is sourced: `echo $ROS_DISTRO`
- Check nodes: `ros2 node list`

### Camera shows black
- Ensure camera is initialized
- Try moving head with arrow keys
- Check camera rendering in simulation window

## Available Scripts

| Script | Purpose | Environment |
|--------|---------|-------------|
| `scripts/stretch_ros2_sim.py` | Main simulation node | simenv_ros2 |
| `scripts/interactive_controller.py` | Interactive CLI controller | simenv_ros2 |
| `scripts/stretch_keyboard_controller.py` | Keyboard controller | simenv_ros2 |
| `tests/test_ros2_communication.py` | Test ROS 2 setup | simenv_ros2 |
| `scripts/verify_setup.py` | Verify environment setup | simenv_ros2 |
