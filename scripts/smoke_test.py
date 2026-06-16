#!/usr/bin/env python3
"""Non-interactive smoke tests (no GUI, no long-running nodes)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def run(label: str, ok: bool, detail: str = "") -> bool:
    mark = "✓" if ok else "✗"
    line = f"{mark} {label}"
    if detail:
        line += f" — {detail}"
    print(line)
    return ok


def main() -> int:
    print("=" * 60)
    print("Stretch 3 Simulation — Smoke Tests")
    print("=" * 60)
    print()

    results: list[bool] = []

    # Python files compile
    py_files = list(ROOT.glob("scripts/*.py"))
    py_files += list(ROOT.glob("src/stretch_sim/*.py"))
    py_files += list(ROOT.glob("tests/*.py"))
    compile_ok = True
    for path in py_files:
        try:
            subprocess.check_call(
                [sys.executable, "-m", "py_compile", str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError:
            compile_ok = False
            print(f"✗ compile failed: {path.relative_to(ROOT)}")
    results.append(run(f"Python syntax ({len(py_files)} files)", compile_ok))

    # Package paths
    try:
        from stretch_sim.paths import get_config_path, get_repo_root, get_xml_path

        results.append(run("stretch_sim.paths", get_repo_root() == ROOT))
        results.append(run("table_world.xml", get_xml_path().is_file()))
        results.append(run("actions.yaml", get_config_path("actions.yaml").is_file()))
    except Exception as exc:
        results.append(run("stretch_sim.paths", False, str(exc)))

    # Anchors
    try:
        from stretch_sim.anchor_utils import load_anchors_from_xml

        anchors = load_anchors_from_xml()
        expected = {"A", "B", "C", "D", "E", "F", "G", "ORIGIN"}
        results.append(run("anchors loaded", set(anchors) == expected, f"{len(anchors)} anchors"))
    except Exception as exc:
        results.append(run("anchors loaded", False, str(exc)))

    # MuJoCo model
    try:
        import mujoco
        from stretch_sim.paths import get_xml_path

        model = mujoco.MjModel.from_xml_path(str(get_xml_path()))
        results.append(
            run("MuJoCo model", True, f"{model.nq} DOF, {model.nu} actuators")
        )
    except ImportError:
        results.append(run("MuJoCo model", False, "mujoco not installed"))
    except Exception as exc:
        results.append(run("MuJoCo model", False, str(exc)))

    # Actions YAML
    try:
        import yaml
        from stretch_sim.paths import get_config_path

        with open(get_config_path("actions.yaml")) as f:
            data = yaml.safe_load(f)
        n_micro = len(data.get("micro_actions", []))
        n_macro = len(data.get("macro_actions", []))
        results.append(run("actions.yaml", n_micro >= 10 and n_macro >= 1, f"{n_micro} micro, {n_macro} macro"))
    except Exception as exc:
        results.append(run("actions.yaml", False, str(exc)))

    # ROS imports (optional)
    if os.environ.get("ROS_DISTRO"):
        try:
            import rclpy  # noqa: F401
            from geometry_msgs.msg import Twist  # noqa: F401
            from sensor_msgs.msg import JointState  # noqa: F401
            from std_msgs.msg import Float64MultiArray, String  # noqa: F401

            results.append(run("ROS 2 imports", True, os.environ["ROS_DISTRO"]))
            try:
                if not rclpy.ok():
                    rclpy.init()
                node = rclpy.create_node("stretch_smoke_test")
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                results.append(run("ROS 2 node create", True))
            except Exception as exc:
                results.append(
                    run(
                        "ROS 2 node create",
                        False,
                        f"{exc} (check ROS install / RMW_IMPLEMENTATION)",
                    )
                )
        except ImportError as exc:
            results.append(run("ROS 2 imports", False, str(exc)))
    else:
        print("○ ROS 2 tests skipped (source /opt/ros/jazzy/setup.bash to enable)")

    print()
    print("=" * 60)
    if all(results):
        print("✅ All smoke tests passed.")
        return 0
    print("❌ Some smoke tests failed.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
