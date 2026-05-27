#!/bin/bash
# Run ROS 2 simulation with conda + ROS environment

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH}"

eval "$(conda shell.bash hook)"
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash

python scripts/stretch_ros2_sim.py
