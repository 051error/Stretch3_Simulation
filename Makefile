.PHONY: help install verify smoke sim controller view test test-ros

ROOT := $(shell pwd)
export PYTHONPATH := $(ROOT)/src:$(PYTHONPATH)
# Fast DDS can fail node creation on some setups; Cyclone DDS is more reliable.
export RMW_IMPLEMENTATION ?= rmw_cyclonedds_cpp

# Use active environment python (conda simenv / simenv_ros2 should be 3.12+ for ROS).
PYTHON ?= python

help:
	@echo "Stretch 3 Simulation"
	@echo ""
	@echo "  make install     Install package in editable mode (recommended)"
	@echo "  make verify      Run setup verification"
	@echo "  make smoke       Quick non-interactive tests (recommended)"
	@echo "  make sim         Start MuJoCo + ROS 2 simulation"
	@echo "  make controller  Start interactive CLI controller"
	@echo "  make view        Open MuJoCo world viewer"
	@echo "  make test        Run ROS 2 communication tests"

install:
	pip install -e .

verify:
	$(PYTHON) scripts/verify_setup.py

smoke:
	$(PYTHON) scripts/smoke_test.py

sim:
	@test -n "$$ROS_DISTRO" || (echo "Source ROS 2 first: source /opt/ros/jazzy/setup.bash" && exit 1)
	$(PYTHON) scripts/stretch_ros2_sim.py

controller:
	@test -n "$$ROS_DISTRO" || (echo "Source ROS 2 first: source /opt/ros/jazzy/setup.bash" && exit 1)
	$(PYTHON) scripts/interactive_controller.py

view:
	$(PYTHON) scripts/view_world.py

test: test-ros

test-ros:
	@test -n "$$ROS_DISTRO" || (echo "Source ROS 2 first: source /opt/ros/jazzy/setup.bash" && exit 1)
	$(PYTHON) tests/test_ros2_communication.py
install:
	@test -n "$$CONDA_DEFAULT_ENV" || echo "Warning: no conda env active — consider activating simenv_ros2 first"
	pip install -e .
