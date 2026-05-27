#!/bin/bash
# Verify Python 3.12 + ROS 2 + MuJoCo imports

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

eval "$(conda shell.bash hook)"
conda activate simenv_ros2 2>/dev/null || conda activate simenv

PYTHON_VERSION=$(python --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

if [[ "$PYTHON_VERSION" == 3.12* ]]; then
    echo "✓ Python 3.12 detected"
else
    echo "✗ Expected Python 3.12"
    exit 1
fi

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

echo ""
echo "Testing imports..."
python -c "import rclpy; print('✓ rclpy')"
python -c "import mujoco; print('✓ mujoco')"

echo ""
echo "Ready: make sim"
