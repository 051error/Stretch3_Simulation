# Setup Guide

Complete setup instructions for the Stretch 2 Simulation Environment.

## Prerequisites

- **Conda** (Miniconda or Anaconda)
- **ROS 2 Jazzy** (for ROS 2 features)
- **Python 3.12**

## Installation

### 1. Clone Repository

```bash
git clone <repository-url>
cd Stretch3_Simulation
```

### 2. Create Environment

```bash
# Create ROS 2 environment
conda env create -f environment/environment_ros2.yml

# Activate environment
conda activate simenv_ros2

# Source ROS 2
source /opt/ros/jazzy/setup.bash
```

### 3. Install Package and Verify

```bash
pip install -e .
make smoke     # quick automated checks (recommended)
make verify    # full environment verification
```

`make smoke` checks Python syntax, repo paths, MuJoCo model loading, `actions.yaml`, and (if ROS is sourced) ROS 2 imports.

`make verify` additionally checks:
- Conda environment activation
- Required Python packages (including Python 3.12)
- Required files and directories

## ROS 2 Setup

### Install ROS 2

Follow the official installation guide:
- **Ubuntu 22.04**: [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- **Ubuntu 24.04**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

### Install Python Packages

```bash
conda activate simenv_ros2
pip install rclpy
```

Note: ROS 2 message packages (`geometry_msgs`, `sensor_msgs`, `std_msgs`) are included with ROS 2 installation.

### Verify ROS 2

```bash
source /opt/ros/jazzy/setup.bash
python -c "import rclpy; print('ROS 2 ready')"
```

## ROS 2 Topics

### Subscribed (Commands)
- `/stretch/cmd_vel` - Base velocity commands
- `/stretch/joint_commands` - Joint position commands
- `/stretch/navigate_to_anchor` - Navigate to anchor
- `/stretch/turn_towards_anchor` - Turn towards anchor
- `/stretch/navigate_to_position` - Navigate to position
- `/stretch/reset_arm` - Reset arm command

### Published (State)
- `/stretch/joint_states` - Current joint positions and velocities
- `/stretch/navigation_active` - Navigation status
- `/stretch/camera/image_raw` - Camera feed

## Environment Management

### Activate Environment

```bash
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash
```

### Update Environment

```bash
conda env update -f environment/environment_ros2.yml --prune
```

### Remove Environment

```bash
conda env remove -n simenv_ros2
```

## Troubleshooting

### "No module named 'rclpy'"
- Ensure ROS 2 is installed
- Source ROS 2: `source /opt/ros/jazzy/setup.bash`
- Install: `pip install rclpy`

### "ImportError: cannot import name 'Twist'"
- Source ROS 2 setup script before running
- Message packages are part of ROS 2 installation

### Topics not appearing
- Verify simulation node is running
- Check ROS 2 is sourced: `echo $ROS_DISTRO`
- List nodes: `ros2 node list`

### `RuntimeError: This member is not been selected` (rclpy)
- Set Cyclone DDS before running sim or tests:
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- The Makefile and `bin/*.sh` scripts set this by default.

### Python version issues
- Ensure using Python 3.12
- Check: `python --version`
- Recreate environment if needed

## Dependencies

### Core
- `mujoco` - Physics simulation
- `mujoco-python-viewer` - Visualization
- `numpy` - Numerical operations
- `opencv-python` - Image processing

### ROS 2
- `rclpy` - ROS 2 Python client
- ROS 2 message packages (via ROS 2 installation)

## Portability

This repository is designed to work on any system:
- All paths are relative
- No hardcoded user-specific paths
- Works on Linux, macOS, Windows (with conda)
- Scripts can be run from any directory

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Stretch 2 Documentation](https://docs.hello-robot.com/)
