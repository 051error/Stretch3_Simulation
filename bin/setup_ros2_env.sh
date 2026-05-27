#!/bin/bash
# Create the ROS 2 conda environment (simenv_ros2)

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "=========================================="
echo "Creating ROS 2 Environment (Python 3.12)"
echo "=========================================="
echo ""
echo "This creates conda env 'simenv_ros2' from environment/environment_ros2.yml"
echo "Your existing 'simenv' environment is not modified."
echo ""

read -p "Continue? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

ENV_FILE="${REPO_ROOT}/environment/environment_ros2.yml"
if [ ! -f "$ENV_FILE" ]; then
    echo "Error: $ENV_FILE not found"
    exit 1
fi

conda env create -f "$ENV_FILE"

echo ""
echo "=========================================="
echo "Environment created successfully"
echo "=========================================="
echo ""
echo "  conda activate simenv_ros2"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  pip install -e ."
echo "  make verify"
echo ""
