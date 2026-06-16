"""Gymnasium environment for Stretch 3 arm reaching and grasping with RL.

This is a standalone MuJoCo environment for fast training — no ROS 2 in the loop.
After training, the policy is deployed through the macro action system via ROS 2.
"""

import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import gymnasium as gym
except ImportError:
    raise ImportError(
        "gymnasium is required. Install with: pip install gymnasium"
    )

import mujoco

# Ensure project src is on path for model resolution
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

from stretch_sim.paths import get_xml_path

# ── joint indices in qpos ──────────────────────────────────────────
LIFT_QPOS = 9       # joint_lift
ARM0_QPOS = 10      # joint_arm_l3
ARM1_QPOS = 11      # joint_arm_l2
ARM2_QPOS = 12      # joint_arm_l1
ARM3_QPOS = 13      # joint_arm_l0
WRIST_QPOS = 14     # joint_wrist_yaw
GRIP_QPOS = 15      # joint_gripper_slide (approx – may be actuator-only)

# ── joint limits ───────────────────────────────────────────────────
LIFT_MIN, LIFT_MAX = -0.5, 0.6
ARM_MIN, ARM_MAX = 0.0, 0.52           # total extension (sum of 4 segments)
WRIST_MIN, WRIST_MAX = -1.75, 4.0
GRIP_MIN, GRIP_MAX = -0.005, 0.04

# ── actuator names matching stretch.xml ────────────────────────────
ACTUATOR_NAMES = [
    "forward", "turn", "lift", "arm_extend", "wrist_yaw",
    "grip", "head_pan", "head_tilt",
]

# ── free-joint indices (base x, y, z, qw, qx, qy, qz) ─────────────
BASE_POS_START = 0
BASE_QUAT_START = 3


class StretchPickEnv(gym.Env):
    """Gymnasium environment: control Stretch 3 arm to pick a tomato.

    Observation space (13 dims, all float32):
        [lift, arm_total, wrist_yaw, gripper,                # 4  joint positions
         ee_x, ee_y, ee_z,                                   # 3  end-effector world pos
         target_x, target_y, target_z,                       # 3  target object world pos
         gripper_open,                                       # 1  is gripper open (binary-like)
         target_dist,                                        # 1  distance ee → target
         vertical_aligned]                                   # 1  vertical alignment flag

    Action space (4 dims, each in [-1, 1]):
        [delta_lift, delta_arm, delta_wrist, gripper_cmd]
    """

    # ── reward config ──────────────────────────────────────────────
    # Design principles:
    #   1. Keep distance penalty light so it doesn't drown sparse rewards
    #   2. Use a "progress bonus" — reward per-step movement toward target
    #   3. Scale grasp/lift/success bonuses high enough for PPO to distinguish
    reward_config = {
        "dist_weight": 0.3,       # per-metre distance penalty (kept small)
        "progress_weight": 5.0,   # +5 per metre moved toward target (instant feedback)
        "align_bonus": 5.0,       # gripper aligned above target (one-shot)
        "grasp_bonus": 15.0,      # gripper closed while aligned
        "lift_bonus": 25.0,       # object lifted off table
        "success_bonus": 50.0,    # full pick-and-lift success
        "action_penalty": 0.005,  # L2 action regularisation
        "time_penalty": 0.003,    # per-step cost (light, allows exploration)
    }

    def __init__(
        self,
        xml_path: Optional[str] = None,
        tomato_name: str = "tomato1",
        max_episode_steps: int = 200,
        render_mode: Optional[str] = None,
    ):
        super().__init__()

        if xml_path is None:
            xml_path = str(get_xml_path())
        self._xml_path = xml_path
        self._tomato_name = tomato_name
        self._max_episode_steps = max_episode_steps
        self.render_mode = render_mode

        # ── load model ─────────────────────────────────────────────
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # ── cache ids ──────────────────────────────────────────────
        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site"
        )
        self.tomato_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, f"{tomato_name}_site"
        )
        self.tomato_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, tomato_name
        )

        self.actuator_ids = {
            name: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            for name in ACTUATOR_NAMES
        }

        # ── spaces ─────────────────────────────────────────────────
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # ── episode state ──────────────────────────────────────────
        self._step_count = 0
        self._was_close = False
        self._was_grasped = False
        self._was_lifted = False
        self._was_success = False
        self._target_pos = None
        self._initial_object_pos = None  # Track object for lift detection

    # ── helpers ────────────────────────────────────────────────────
    def _get_ee_pos(self) -> np.ndarray:
        """Current end-effector world position."""
        mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.ee_site_id].copy()

    def _get_tomato_pos(self) -> np.ndarray:
        """Hardcoded target position (saved at reset, immutable by physics collisions)."""
        return self._target_pos.copy()

    def _get_joint_positions(self) -> np.ndarray:
        """Return [lift, arm_total, wrist_yaw, gripper] from ctrl state (what we commanded)."""
        return np.array([
            float(self.data.ctrl[self.actuator_ids["lift"]]),
            float(self.data.ctrl[self.actuator_ids["arm_extend"]]),
            float(self.data.ctrl[self.actuator_ids["wrist_yaw"]]),
            float(self.data.ctrl[self.actuator_ids["grip"]]),
        ], dtype=np.float32)

    def _apply_action(self, action: np.ndarray):
        """Apply action deltas to actuators, respecting limits.

        Both reads and writes use ctrl values (not qpos). General actuators
        accept ctrl values in joint-space range (e.g. lift ∈ [-0.5, 0.6]),
        and MuJoCo's internal position servo pulls the joint to that target.
        """
        current = self._get_joint_positions()
        # action in [-1, 1], scale to meaningful deltas
        # arm_extend gets a larger scale — the joints move slowly through
        # the tendon transmission so a bigger step is needed for exploration
        scale = np.array([0.06, 0.08, 0.12, 0.005], dtype=np.float32)
        deltas = action * scale

        new_lift = np.clip(current[0] + deltas[0], LIFT_MIN, LIFT_MAX)
        new_arm = np.clip(current[1] + deltas[1], ARM_MIN, ARM_MAX)
        new_wrist = np.clip(current[2] + deltas[2], WRIST_MIN, WRIST_MAX)
        new_grip = np.clip(current[3] + deltas[3], GRIP_MIN, GRIP_MAX)

        # Write to ctrl state
        self.data.ctrl[self.actuator_ids["lift"]] = new_lift
        self.data.ctrl[self.actuator_ids["arm_extend"]] = new_arm
        self.data.ctrl[self.actuator_ids["wrist_yaw"]] = new_wrist
        self.data.ctrl[self.actuator_ids["grip"]] = new_grip

    def _get_obs(self) -> np.ndarray:
        """Build observation vector."""
        joints = self._get_joint_positions()
        ee = self._get_ee_pos()
        tomato = self._get_tomato_pos()
        dist = np.linalg.norm(ee - tomato)
        gripper_open = 1.0 if joints[3] > 0.02 else 0.0
        vertical_aligned = 1.0 if abs(ee[0] - tomato[0]) < 0.08 and abs(ee[1] - tomato[1]) < 0.08 else 0.0

        return np.array([
            joints[0], joints[1], joints[2], joints[3],
            ee[0], ee[1], ee[2],
            tomato[0], tomato[1], tomato[2],
            gripper_open,
            dist,
            vertical_aligned,
        ], dtype=np.float32)

    # ── pre-computed tomato world positions (from table_world.xml) ─
    _TOMATO_POSITIONS = {
        "tomato1": np.array([0.43, 4.0, 0.65], dtype=np.float32),
        "tomato2": np.array([0.65, 4.0, 0.65], dtype=np.float32),
        "tomato3": np.array([0.87, 4.0, 0.65], dtype=np.float32),
    }

    # ── gym interface ──────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Reset MuJoCo to initial state
        self.data = mujoco.MjData(self.model)
        self.data.ctrl[:] = 0.0

        # Position robot 0.22–0.32 m in front of the tomato (verified reachable to 2.5 cm).
        # The Stretch arm extends in local -Y, so the robot faces -X (west)
        # so that the arm extends toward +Y (north, toward the table).
        target = self._TOMATO_POSITIONS[self._tomato_name]
        base_y = target[1] - self.np_random.uniform(0.22, 0.32)
        base_x = target[0] + self.np_random.uniform(-0.06, 0.06)
        self.data.qpos[0] = base_x
        self.data.qpos[1] = base_y
        self.data.qpos[2] = 0.0

        # Face -X (west) with small perturbation: arm local -Y → world +Y (north, toward table)
        base_yaw = math.pi + self.np_random.uniform(-0.12, 0.12)
        self.data.qpos[3] = math.cos(base_yaw / 2.0)
        self.data.qpos[4] = 0.0
        self.data.qpos[5] = 0.0
        self.data.qpos[6] = math.sin(base_yaw / 2.0)

        # Initialise arm actuators via ctrl after FK
        mujoco.mj_forward(self.model, self.data)
        # Start with arm raised + retracted → EE far (~20-30cm), RL must learn
        # to lower lift and extend arm to get within 10cm of the target.
        self.data.ctrl[self.actuator_ids["lift"]] = self.np_random.uniform(0.30, 0.50)
        self.data.ctrl[self.actuator_ids["arm_extend"]] = self.np_random.uniform(0.0, 0.08)
        self.data.ctrl[self.actuator_ids["wrist_yaw"]] = self.np_random.uniform(0.5, 1.5)
        self.data.ctrl[self.actuator_ids["grip"]] = 0.04  # gripper open
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)

        # Cache the fixed target position (hardcoded — avoids physics pushing the tomato off)
        self._target_pos = self._TOMATO_POSITIONS[self._tomato_name].copy()

        # Forward the model to update site positions after setting qpos and ctrl targets
        mujoco.mj_forward(self.model, self.data)

        # Record initial physical tomato position (for lift detection)
        self._initial_object_pos = self.data.xpos[self.tomato_body_id].copy()

        # Reset episode state
        self._step_count = 0
        self._was_close = False
        self._was_grasped = False
        self._was_lifted = False
        self._was_success = False
        self._prev_dist = float(np.linalg.norm(self._get_ee_pos() - self._target_pos))

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        self._step_count += 1

        # Apply action
        self._apply_action(np.clip(action, -1.0, 1.0))

        # Step simulation — enough substeps for actuators to converge to ctrl targets
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        # Get observations
        obs = self._get_obs()
        ee_pos = self._get_ee_pos()
        tomato_pos = self._get_tomato_pos()
        joints = self._get_joint_positions()

        dist = float(np.linalg.norm(ee_pos - tomato_pos))
        gripper_closed = joints[3] < 0.01
        object_lifted = self.data.xpos[self.tomato_body_id][2] > self._initial_object_pos[2] + 0.05

        # ── rewards ────────────────────────────────────────────────
        rc = self.reward_config
        reward = 0.0

        # 1. Light distance penalty (kept small, doesn't dominate)
        reward -= rc["dist_weight"] * dist

        # 2. Progress bonus — instant per-step feedback toward target
        if self._prev_dist is not None:
            dist_change = self._prev_dist - dist  # >0 = moving closer
            reward += rc["progress_weight"] * dist_change
        self._prev_dist = dist

        # 3. Alignment bonus — one-shot: gripper first reaches the target zone
        horizontal_dist = float(np.linalg.norm(ee_pos[:2] - tomato_pos[:2]))
        vertical_dist = abs(ee_pos[2] - tomato_pos[2])
        if not self._was_close and horizontal_dist < 0.08 and vertical_dist < 0.06:
            reward += rc["align_bonus"]
            self._was_close = True

        # 4. Grasp bonus — one-shot: first time gripper closes while aligned
        if (self._was_close and not self._was_grasped
                and gripper_closed and horizontal_dist < 0.10):
            reward += rc["grasp_bonus"]
            self._was_grasped = True

        # 5. Lift bonus — one-shot: first time object is lifted after grasp
        if self._was_grasped and not self._was_lifted and object_lifted:
            reward += rc["lift_bonus"]
            self._was_lifted = True

        # 6. Success bonus — one-shot: arm raised with object grasped
        if (self._was_lifted and not self._was_success
                and object_lifted and joints[0] > 0.2):
            reward += rc["success_bonus"]
            self._was_success = True

        # 7. Action penalty — light smoothness regularisation
        reward -= rc["action_penalty"] * float(np.sum(np.square(action)))

        # 8. Per-step cost — very light time pressure
        reward -= rc["time_penalty"]

        # ── termination ────────────────────────────────────────────
        terminated = False
        truncated = False

        # Success: grasped and lifted with arm raised (one-shot detection)
        success = self._was_success
        if success:
            terminated = True

        # Timeout
        if self._step_count >= self._max_episode_steps:
            truncated = True

        # Episode too far (robot drove gripper into unreachable zone)
        if dist > 2.0:
            truncated = True

        info = {
            "dist": dist,
            "horizontal_dist": horizontal_dist,
            "vertical_dist": vertical_dist,
            "gripper_closed": gripper_closed,
            "object_lifted": object_lifted,
            "was_close": self._was_close,
            "was_grasped": self._was_grasped,
            "success": success,
        }

        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            # Use a passive viewer if available (one-shot render)
            try:
                with mujoco.viewer.launch_passive(
                    self.model, self.data, show_left_ui=False, show_right_ui=False
                ) as viewer:
                    viewer.sync()
            except Exception:
                pass

    def close(self):
        pass


# ═══════════════════════════════════════════════════════════════════
# Stage 1: Approach — move end-effector near the tomato (no gripper)
# ═══════════════════════════════════════════════════════════════════

class StretchReachEnv(gym.Env):
    """Stage 1: control lift, arm, wrist to bring the end-effector near the tomato.

    Observation (10 dims):
        [lift, arm_total, wrist_yaw,
         ee_x, ee_y, ee_z,
         target_x, target_y, target_z,
         dist]

    Action (3 dims in [-1,1]):
        [delta_lift, delta_arm, delta_wrist]
    """

    reward_config = {
        "dist_weight": 0.1,         # light per-metre distance penalty (was 0.5)
        "progress_weight": 50.0,    # dominant per-step shaping toward target (was 8.0)
        "reach_bonus": 30.0,        # one-shot: EE within 10 cm of target
        "action_penalty": 0.01,     # smoothness regularisation
        "time_penalty": 0.005,      # light time pressure
    }

    # Re-use shared helpers via a shared base init utility
    _TOMATO_POSITIONS = {
        "tomato1": np.array([0.43, 4.0, 0.65], dtype=np.float32),
        "tomato2": np.array([0.65, 4.0, 0.65], dtype=np.float32),
        "tomato3": np.array([0.87, 4.0, 0.65], dtype=np.float32),
    }

    def __init__(self, xml_path=None, tomato_name="tomato1", max_episode_steps=120,
                 render_mode=None):
        super().__init__()
        if xml_path is None:
            xml_path = str(get_xml_path())
        self._xml_path = xml_path
        self._tomato_name = tomato_name
        self._max_episode_steps = max_episode_steps
        self.render_mode = render_mode

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self.tomato_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, tomato_name)

        self.actuator_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        }

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        self._step_count = 0
        self._reached = False
        self._target_pos = None

    # ── helpers (mirror StretchPickEnv) ───────────────────────────
    def _get_ee_pos(self):
        mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.ee_site_id].copy()

    def _get_joint_positions(self):
        return np.array([
            float(self.data.ctrl[self.actuator_ids["lift"]]),
            float(self.data.ctrl[self.actuator_ids["arm_extend"]]),
            float(self.data.ctrl[self.actuator_ids["wrist_yaw"]]),
        ], dtype=np.float32)

    def _apply_reach_action(self, action):
        current = self._get_joint_positions()
        scale = np.array([0.06, 0.08, 0.12], dtype=np.float32)
        deltas = np.clip(action, -1.0, 1.0).astype(np.float32) * scale

        self.data.ctrl[self.actuator_ids["lift"]] = (
            np.clip(current[0] + deltas[0], LIFT_MIN, LIFT_MAX))
        self.data.ctrl[self.actuator_ids["arm_extend"]] = (
            np.clip(current[1] + deltas[1], ARM_MIN, ARM_MAX))
        self.data.ctrl[self.actuator_ids["wrist_yaw"]] = (
            np.clip(current[2] + deltas[2], WRIST_MIN, WRIST_MAX))
        self.data.ctrl[self.actuator_ids["grip"]] = 0.04  # keep gripper open

    def _get_obs(self):
        joints = self._get_joint_positions()
        ee = self._get_ee_pos()
        dist = float(np.linalg.norm(ee - self._target_pos))
        return np.array([
            joints[0], joints[1], joints[2],
            ee[0], ee[1], ee[2],
            self._target_pos[0], self._target_pos[1], self._target_pos[2],
            dist,
        ], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.data = mujoco.MjData(self.model)
        self.data.ctrl[:] = 0.0

        target = self._TOMATO_POSITIONS[self._tomato_name]
        base_y = target[1] - self.np_random.uniform(0.22, 0.32)
        base_x = target[0] + self.np_random.uniform(-0.06, 0.06)
        self.data.qpos[0] = base_x
        self.data.qpos[1] = base_y
        self.data.qpos[2] = 0.0
        base_yaw = math.pi + self.np_random.uniform(-0.12, 0.12)
        self.data.qpos[3] = math.cos(base_yaw / 2.0)
        self.data.qpos[4] = 0.0
        self.data.qpos[5] = 0.0
        self.data.qpos[6] = math.sin(base_yaw / 2.0)

        mujoco.mj_forward(self.model, self.data)
        # Start with arm raised + retracted → EE far (~20-30cm), RL must learn
        # to lower lift and extend arm to get within 10cm of the target.
        self.data.ctrl[self.actuator_ids["lift"]] = self.np_random.uniform(0.30, 0.50)
        self.data.ctrl[self.actuator_ids["arm_extend"]] = self.np_random.uniform(0.0, 0.08)
        self.data.ctrl[self.actuator_ids["wrist_yaw"]] = self.np_random.uniform(0.5, 1.5)
        self.data.ctrl[self.actuator_ids["grip"]] = 0.04
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)

        self._target_pos = self._TOMATO_POSITIONS[self._tomato_name].copy()
        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        self._reached = False
        self._prev_dist = float(np.linalg.norm(self._get_ee_pos() - self._target_pos))
        return self._get_obs(), {}

    def step(self, action):
        self._step_count += 1
        self._apply_reach_action(action)

        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        ee_pos = self._get_ee_pos()
        dist = float(np.linalg.norm(ee_pos - self._target_pos))

        rc = self.reward_config
        reward = 0.0

        # 1. Distance penalty
        reward -= rc["dist_weight"] * dist

        # 2. Progress bonus — strong shaping signal
        if self._prev_dist is not None:
            reward += rc["progress_weight"] * (self._prev_dist - dist)
        self._prev_dist = dist

        # 3. Reach bonus — one-shot when EE within 5 cm
        if not self._reached and dist < 0.10:
            reward += rc["reach_bonus"]
            self._reached = True

        # 4. Action regularisation & time cost
        reward -= rc["action_penalty"] * float(np.sum(np.square(action)))
        reward -= rc["time_penalty"]

        terminated = self._reached
        truncated = (self._step_count >= self._max_episode_steps) or (dist > 2.0)

        info = {"dist": dist, "reached": self._reached}
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            try:
                with mujoco.viewer.launch_passive(
                    self.model, self.data, show_left_ui=False, show_right_ui=False
                ) as viewer:
                    viewer.sync()
            except Exception:
                pass

    def close(self):
        pass


# ═══════════════════════════════════════════════════════════════════
# Stage 2: Grasp + Lift — close gripper around tomato and raise arm
# ═══════════════════════════════════════════════════════════════════

class StretchGraspEnv(gym.Env):
    """Stage 2: grasp the tomato (already near EE) and lift it.

    Starts with EE already close to the target (simulates Stage 1 output).

    Observation (11 dims):
        [lift, arm_total, wrist_yaw, gripper,
         ee_x, ee_y, ee_z,
         target_x, target_y, target_z,
         dist]

    Action (4 dims in [-1,1]):
        [delta_lift, delta_arm, delta_wrist, gripper_cmd]
    """

    reward_config = {
        "dist_weight": 0.2,         # very light — EE is already near target
        "progress_weight": 3.0,     # mild per-step shaping
        "align_bonus": 10.0,        # one-shot: EE within 5 cm above target
        "grasp_bonus": 20.0,        # one-shot: gripper closed while aligned
        "lift_bonus": 30.0,         # one-shot: object lifted off table
        "success_bonus": 50.0,      # one-shot: full pick-and-lift
        "action_penalty": 0.005,    # light smoothness regularisation
        "time_penalty": 0.002,
    }

    _TOMATO_POSITIONS = {
        "tomato1": np.array([0.43, 4.0, 0.65], dtype=np.float32),
        "tomato2": np.array([0.65, 4.0, 0.65], dtype=np.float32),
        "tomato3": np.array([0.87, 4.0, 0.65], dtype=np.float32),
    }

    def __init__(self, xml_path=None, tomato_name="tomato1", max_episode_steps=150,
                 render_mode=None):
        super().__init__()
        if xml_path is None:
            xml_path = str(get_xml_path())
        self._xml_path = xml_path
        self._tomato_name = tomato_name
        self._max_episode_steps = max_episode_steps
        self.render_mode = render_mode

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self.tomato_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, tomato_name)

        self.actuator_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        }

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

        self._step_count = 0
        self._was_aligned = False
        self._was_grasped = False
        self._was_lifted = False
        self._was_success = False
        self._target_pos = None
        self._initial_object_pos = None

    def _get_ee_pos(self):
        mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.ee_site_id].copy()

    def _get_joint_positions(self):
        return np.array([
            float(self.data.ctrl[self.actuator_ids["lift"]]),
            float(self.data.ctrl[self.actuator_ids["arm_extend"]]),
            float(self.data.ctrl[self.actuator_ids["wrist_yaw"]]),
            float(self.data.ctrl[self.actuator_ids["grip"]]),
        ], dtype=np.float32)

    def _apply_action(self, action):
        current = self._get_joint_positions()
        scale = np.array([0.04, 0.05, 0.08, 0.004], dtype=np.float32)
        deltas = np.clip(action, -1.0, 1.0).astype(np.float32) * scale

        self.data.ctrl[self.actuator_ids["lift"]] = (
            np.clip(current[0] + deltas[0], LIFT_MIN, LIFT_MAX))
        self.data.ctrl[self.actuator_ids["arm_extend"]] = (
            np.clip(current[1] + deltas[1], ARM_MIN, ARM_MAX))
        self.data.ctrl[self.actuator_ids["wrist_yaw"]] = (
            np.clip(current[2] + deltas[2], WRIST_MIN, WRIST_MAX))
        self.data.ctrl[self.actuator_ids["grip"]] = (
            np.clip(current[3] + deltas[3], GRIP_MIN, GRIP_MAX))

    def _get_obs(self):
        joints = self._get_joint_positions()
        ee = self._get_ee_pos()
        dist = float(np.linalg.norm(ee - self._target_pos))
        return np.array([
            joints[0], joints[1], joints[2], joints[3],
            ee[0], ee[1], ee[2],
            self._target_pos[0], self._target_pos[1], self._target_pos[2],
            dist,
        ], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.data = mujoco.MjData(self.model)
        self.data.ctrl[:] = 0.0

        target = self._TOMATO_POSITIONS[self._tomato_name]
        # Same base placement as approach env — the optimal distance for the
        # Stretch arm to reach the table.  Two stages share the same start
        # distribution but optimise different objectives:
        #   Stage 1: minimise EE→target distance (positioning)
        #   Stage 2: close gripper + lift object (grasping)
        base_y = target[1] - self.np_random.uniform(0.22, 0.32)
        base_x = target[0] + self.np_random.uniform(-0.06, 0.06)
        self.data.qpos[0] = base_x
        self.data.qpos[1] = base_y
        self.data.qpos[2] = 0.0
        base_yaw = math.pi + self.np_random.uniform(-0.12, 0.12)
        self.data.qpos[3] = math.cos(base_yaw / 2.0)
        self.data.qpos[4] = 0.0
        self.data.qpos[5] = 0.0
        self.data.qpos[6] = math.sin(base_yaw / 2.0)

        mujoco.mj_forward(self.model, self.data)

        # Post-approach state: arm near optimal config (lift low, arm extended,
        # wrist in sweet spot).  EE should be within ~5-15 cm of target.
        self.data.ctrl[self.actuator_ids["lift"]] = self.np_random.uniform(-0.05, 0.15)
        self.data.ctrl[self.actuator_ids["arm_extend"]] = self.np_random.uniform(0.30, 0.48)
        self.data.ctrl[self.actuator_ids["wrist_yaw"]] = self.np_random.uniform(0.7, 2.0)
        self.data.ctrl[self.actuator_ids["grip"]] = 0.04  # open
        for _ in range(30):
            mujoco.mj_step(self.model, self.data)

        self._target_pos = self._TOMATO_POSITIONS[self._tomato_name].copy()
        mujoco.mj_forward(self.model, self.data)
        self._initial_object_pos = self.data.xpos[self.tomato_body_id].copy()

        self._step_count = 0
        self._was_aligned = False
        self._was_grasped = False
        self._was_lifted = False
        self._was_success = False
        self._prev_dist = float(np.linalg.norm(self._get_ee_pos() - self._target_pos))
        return self._get_obs(), {}

    def step(self, action):
        self._step_count += 1
        self._apply_action(action)

        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        ee_pos = self._get_ee_pos()
        joints = self._get_joint_positions()
        dist = float(np.linalg.norm(ee_pos - self._target_pos))
        gripper_closed = joints[3] < 0.01
        object_lifted = (self.data.xpos[self.tomato_body_id][2]
                         > self._initial_object_pos[2] + 0.03)
        horizontal_dist = float(np.linalg.norm(ee_pos[:2] - self._target_pos[:2]))
        vertical_dist = abs(ee_pos[2] - self._target_pos[2])

        rc = self.reward_config
        reward = 0.0

        # 1. Light distance penalty
        reward -= rc["dist_weight"] * dist

        # 2. Progress bonus
        if self._prev_dist is not None:
            reward += rc["progress_weight"] * (self._prev_dist - dist)
        self._prev_dist = dist

        # 3. Align bonus — one-shot: EE within 5 cm above target
        if not self._was_aligned and horizontal_dist < 0.05 and vertical_dist < 0.05:
            reward += rc["align_bonus"]
            self._was_aligned = True

        # 4. Grasp bonus — one-shot: close gripper while aligned
        if (self._was_aligned and not self._was_grasped
                and gripper_closed and horizontal_dist < 0.08):
            reward += rc["grasp_bonus"]
            self._was_grasped = True

        # 5. Lift bonus — one-shot: object leaves table
        if self._was_grasped and not self._was_lifted and object_lifted:
            reward += rc["lift_bonus"]
            self._was_lifted = True

        # 6. Success — one-shot: full pick with arm raised
        if (self._was_lifted and not self._was_success
                and object_lifted and joints[0] > 0.15):
            reward += rc["success_bonus"]
            self._was_success = True

        # 7. Action & time penalties
        reward -= rc["action_penalty"] * float(np.sum(np.square(action)))
        reward -= rc["time_penalty"]

        terminated = self._was_success
        truncated = (self._step_count >= self._max_episode_steps) or (dist > 2.0)

        info = {
            "dist": dist, "gripper_closed": gripper_closed,
            "object_lifted": object_lifted, "success": self._was_success,
        }
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            try:
                with mujoco.viewer.launch_passive(
                    self.model, self.data, show_left_ui=False, show_right_ui=False
                ) as viewer:
                    viewer.sync()
            except Exception:
                pass

    def close(self):
        pass
