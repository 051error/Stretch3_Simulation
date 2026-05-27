"""Console entry points (see pyproject.toml [project.scripts])."""

import sys
from pathlib import Path


def _scripts_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "scripts"


def _import_script_module(name: str):
    scripts = str(_scripts_dir())
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    return __import__(name)


def run_simulation(args=None):
    _import_script_module("stretch_ros2_sim").main(args)


def run_controller(args=None):
    _import_script_module("interactive_controller").main(args)


def run_verify():
    _import_script_module("verify_setup").main()
