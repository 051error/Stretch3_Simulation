import sys
from pathlib import Path

_src = Path(__file__).resolve().parents[1] / "src"
if _src.is_dir() and str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

import mujoco
import mujoco.viewer

from stretch_sim.paths import get_xml_path

xml_path = str(get_xml_path())
model = mujoco.MjModel.from_xml_path(xml_path)

data = mujoco.MjData(model)
base_link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
print(f"base_link body ID: {base_link_id}")
data.qpos[0:3] = [0, 4, 0.1]
data.qpos[3:7] = [1, 0, 0, 0]

mujoco.mj_forward(model, data)
print("=" * 50)
print("WORLD VIEWER")
print("=" * 50)
print(f"Bodies: {model.nbody}")
print(f"Geoms: {model.ngeom}")
print("\nPress ESC to exit")
print("=" * 50)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [10, 0, 1]
    viewer.cam.distance = 30
    viewer.cam.elevation = -15
    viewer.cam.azimuth = 90

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
