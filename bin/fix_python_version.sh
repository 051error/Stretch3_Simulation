#!/bin/bash
# Script to fix Python version mismatch for ROS 2

echo "=========================================="
echo "Fixing Python Version for ROS 2"
echo "=========================================="
echo ""
echo "This will update your conda environment to Python 3.12"
echo "to match ROS 2 Jazzy requirements."
echo ""
read -p "Continue? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

# Activate environment
eval "$(conda shell.bash hook)"
conda activate simenv

echo "Updating Python to 3.12..."
conda install python=3.12 -y

echo ""
echo "Reinstalling packages that need Python 3.12..."
pip install --force-reinstall mujoco mujoco-python-viewer numpy scipy pynput click

echo ""
echo "=========================================="
echo "Update complete!"
echo "=========================================="
echo ""
echo "Now you can run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  python stretch_ros2_sim.py"
echo ""

