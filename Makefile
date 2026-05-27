.PHONY: help install verify sim controller view test

ROOT := $(shell pwd)
export PYTHONPATH := $(ROOT)/src:$(PYTHONPATH)

help:
	@echo "Stretch 3 Simulation"
	@echo ""
	@echo "  make install     Install package in editable mode (recommended)"
	@echo "  make verify      Run setup verification"
	@echo "  make sim         Start MuJoCo + ROS 2 simulation"
	@echo "  make controller  Start interactive CLI controller"
	@echo "  make view        Open MuJoCo world viewer"
	@echo "  make test        Run ROS 2 communication tests"

install:
	pip install -e .

verify:
	python scripts/verify_setup.py

sim:
	python scripts/stretch_ros2_sim.py

controller:
	python scripts/interactive_controller.py

view:
	python scripts/view_world.py

test:
	python tests/test_ros2_communication.py
