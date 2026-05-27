#!/usr/bin/env python
"""
Setup verification script.
Run this to verify that your environment is set up correctly.
"""
import sys
from pathlib import Path

_src = Path(__file__).resolve().parents[1] / "src"
if _src.is_dir() and str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

from stretch_sim.paths import get_path, get_xml_path


def check_conda_env():
    """Check if conda environment is activated."""
    import os

    conda_env = os.environ.get("CONDA_DEFAULT_ENV", "")
    for name in ("simenv_ros2", "simenv"):
        if conda_env == name:
            print(f"✓ Conda environment '{name}' is activated")
            return True
    print(f"✗ Expected conda env 'simenv_ros2' or 'simenv' (current: {conda_env!r})")
    print("  Run: conda activate simenv_ros2")
    return False


def check_imports():
    """Check if all required packages can be imported."""
    required_packages = [
        "mujoco",
        "mujoco.viewer",
        "numpy",
        "pynput",
        "click",
        "trimesh",
    ]

    failed = []
    for package in required_packages:
        try:
            __import__(package)
            print(f"✓ {package}")
        except ImportError as e:
            print(f"✗ {package} - {e}")
            failed.append(package)

    return len(failed) == 0


def check_files():
    """Check if required files exist."""
    required = [
        get_xml_path(),
        get_path("models", "stretch.xml"),
        get_path("meshes", "table_fixed.obj"),
        get_path("meshes", "plate.obj"),
        get_path("config", "actions.yaml"),
    ]

    missing = []
    for path in required:
        rel = path.relative_to(get_path())
        if path.is_file():
            print(f"✓ {rel}")
        else:
            print(f"✗ {rel} - NOT FOUND")
            missing.append(rel)

    return len(missing) == 0


def check_model_loading():
    """Check if the MuJoCo model can be loaded."""
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(get_xml_path()))
        print(f"✓ MuJoCo model loads successfully ({model.nq} DOF, {model.nu} actuators)")
        return True
    except Exception as e:
        print(f"✗ Failed to load MuJoCo model: {e}")
        return False


def main():
    print("=" * 60)
    print("Stretch 3 Simulation — Setup Verification")
    print("=" * 60)
    print()

    results = []

    print("1. Checking conda environment...")
    results.append(check_conda_env())
    print()

    print("2. Checking Python packages...")
    results.append(check_imports())
    print()

    print("3. Checking required files...")
    results.append(check_files())
    print()

    print("4. Checking MuJoCo model loading...")
    results.append(check_model_loading())
    print()

    print("=" * 60)
    if all(results):
        print("✅ All checks passed! Your environment is ready.")
        print("   Run simulation:  python scripts/stretch_ros2_sim.py")
        print("   Or after install: stretch-sim")
    else:
        print("❌ Some checks failed. Please fix the issues above.")
        sys.exit(1)
    print("=" * 60)


if __name__ == "__main__":
    main()
