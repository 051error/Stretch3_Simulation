#!/usr/bin/env python3
"""Generate 2D lidar rangefinders for the Stretch MuJoCo model.

Regenerates the N ``laser_site_*`` sites and N ``laser_*`` rangefinders that
make up a 2D lidar sweeping the horizontal plane from -pi to pi at 4-degree
resolution, and inlines them into ``models/stretch.xml`` (also writes standalone
fragments for reference). The operation is idempotent: any previously generated
sites/rangefinders are removed first.

Site orientation is built from an axis-angle quaternion: MuJoCo's intrinsic-XYZ
``euler`` cannot yaw a site's z-axis (the z rotation is applied innermost), so a
quaternion is required to point the +z ray direction at an arbitrary azimuth.

MuJoCo rangefinder semantics (verified against MuJoCo 3.2.0):
  * the ray travels along the site's local +z axis
  * an unobstructed ray returns -1.0
  * the sensor ignores the body it is attached to, but still hits other robot
    bodies (the mast at ~0.13-0.15 m); the ROS node filters those as self-hits.
"""

import math
import re
from pathlib import Path

import mujoco
import numpy as np

N = 90  # number of rays (4-degree resolution over a full circle)
MODELS_DIR = Path(__file__).resolve().parents[1] / "models"


def site_quat(theta):
    """Quaternion [w, x, y, z] rotating the +z axis to horizontal azimuth theta."""
    # Rotate (0,0,1) onto (cos theta, sin theta, 0) by rotating pi/2 about the
    # in-plane axis n = z_hat x d = (-sin theta, cos theta, 0).
    axis = np.array([-math.sin(theta), math.cos(theta), 0.0])
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, axis, math.pi / 2)
    return q


def build_fragments():
    """Build the site and sensor fragment text."""
    sites = []
    sensors = []
    for i in range(N):
        theta = -math.pi + i * (2 * math.pi / N)
        q = site_quat(theta)
        sites.append(
            f'      <site name="laser_site_{i}" pos="0 0 0" '
            f'quat="{q[0]:.10f} {q[1]:.10f} {q[2]:.10f} {q[3]:.10f}" '
            f'size="0.005" rgba="1 0 0 0.3"/>'
        )
        sensors.append(f'    <rangefinder name="laser_{i}" site="laser_site_{i}"/>')
    return "\n".join(sites) + "\n", "\n".join(sensors) + "\n"


def main():
    sites, sensors = build_fragments()
    stretch_xml = MODELS_DIR / "stretch.xml"
    text = stretch_xml.read_text()

    # Strip any previously generated sites/rangefinders and their markers.
    text = re.sub(r'[ \t]*<site name="laser_site_\d+"[^\n]*\n', '', text)
    text = re.sub(r'[ \t]*<rangefinder name="laser_\d+"[^\n]*\n', '', text)
    text = re.sub(r'[ \t]*<!-- (BEGIN|END)_LASER_SITES -->\n', '', text)
    text = re.sub(r'[ \t]*<!-- (BEGIN|END)_LASER_SENSORS -->\n', '', text)

    # Insert the site block after the laser body's collision geom.
    site_block = (
        "      <!-- BEGIN_LASER_SITES -->\n" + sites + "      <!-- END_LASER_SITES -->\n"
    )
    site_anchor = '        <geom mesh="laser" class="collision"/>\n'
    if site_anchor not in text:
        raise SystemExit("ERROR: laser collision geom anchor not found in stretch.xml")
    text = text.replace(site_anchor, site_anchor + site_block, 1)

    # Insert the sensor block inside the <sensor> element (create it if needed).
    sensor_block = (
        "    <!-- BEGIN_LASER_SENSORS -->\n" + sensors + "    <!-- END_LASER_SENSORS -->\n"
    )
    if '  <sensor>\n' in text:
        text = text.replace('  <sensor>\n', '  <sensor>\n' + sensor_block, 1)
    else:
        act_anchor = '  </actuator>\n'
        if act_anchor not in text:
            raise SystemExit("ERROR: </actuator> anchor not found in stretch.xml")
        text = text.replace(
            act_anchor, act_anchor + '  <sensor>\n' + sensor_block + '  </sensor>\n', 1
        )

    stretch_xml.write_text(text)
    (MODELS_DIR / "laser_sites.xml").write_text(sites)
    (MODELS_DIR / "laser_rangefinders.xml").write_text(sensors)
    print(f"Regenerated {N} sites and {N} rangefinders in models/stretch.xml")


if __name__ == "__main__":
    main()
