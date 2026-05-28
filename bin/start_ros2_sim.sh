#!/bin/bash
# Quick start: MuJoCo simulation with ROS 2 communication

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 2 not sourced. Attempting to source..."
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        echo "Sourced ROS 2 Jazzy"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "Sourced ROS 2 Humble"
    else
        echo "ERROR: ROS 2 not found. See docs/SETUP.md"
        exit 1
    fi
fi

if command -v conda &>/dev/null; then
    eval "$(conda shell.bash hook)"
    if conda env list | grep -q "simenv_ros2"; then
        conda activate simenv_ros2
    elif conda env list | grep -q "simenv"; then
        conda activate simenv
    fi
fi

echo "=========================================="
echo "Stretch 3 ROS 2 Simulation"
echo "=========================================="
echo "ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "Repo: $REPO_ROOT"
echo ""
echo "In another terminal:"
echo "  make controller"
echo "  # or: python scripts/interactive_controller.py"
echo "=========================================="
echo ""

python scripts/stretch_ros2_sim.py
