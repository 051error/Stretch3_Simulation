#!/usr/bin/env python3
"""RL inference node — runs a trained PPO policy to control the Stretch arm.

Two-stage architecture:
  Stage 1 (approach): move EE near the tomato — outputs (lift, arm, wrist) deltas
  Stage 2 (grasp):   close gripper and lift — outputs 4-dim deltas incl. gripper

Invoked by rl_get_tomato via subprocess. Exits when the stage succeeds or times out.

Usage:
    # Train first:
    python scripts/train_rl_arm.py --stage approach --target tomato1 --timesteps 300000
    python scripts/train_rl_arm.py --stage grasp    --target tomato1 --timesteps 300000

    # Inference (called automatically by the macro):
    python scripts/rl_inference_arm.py --stage approach --target tomato1 --max-steps 200
    python scripts/rl_inference_arm.py --stage grasp    --target tomato1 --max-steps 300
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import mujoco
from stretch_sim.paths import get_xml_path

# ── joint indices ──────────────────────────────────────────────────
LIFT_QPOS = 9
ARM0_QPOS = 10
WRIST_QPOS = 14
GRIP_QPOS = 15

# ── known tomato positions ─────────────────────────────────────────
KNOWN_TOMATO_POSITIONS = {
    "tomato1": np.array([0.43, 4.0, 0.65], dtype=np.float32),
    "tomato2": np.array([0.65, 4.0, 0.65], dtype=np.float32),
    "tomato3": np.array([0.87, 4.0, 0.65], dtype=np.float32),
}

JOINT_ORDER = ["lift", "arm_extend", "wrist_yaw", "gripper", "head_pan", "head_tilt"]

# ── obs sizes per stage ────────────────────────────────────────────
_OBS_DIMS = {"approach": 10, "grasp": 11}
_ACT_DIMS = {"approach": 3, "grasp": 4}


class RLInferenceNode(Node):
    """ROS 2 node that runs trained RL policy for a single stage."""

    def __init__(self, model_path: str, stage: str, tomato_name: str,
                 max_steps: int = 2000, step_delay: float = 0.05):
        super().__init__(f"rl_{stage}_inference")

        self._stage = stage
        self._max_steps = max_steps
        self._step_delay = step_delay
        self._step_count = 0
        self._done = False
        self._success = False

        # ── load MuJoCo model for FK ───────────────────────────────
        xml_path = str(get_xml_path())
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.ee_site_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self._tomato_pos = KNOWN_TOMATO_POSITIONS.get(
            tomato_name, KNOWN_TOMATO_POSITIONS["tomato1"])

        # ── load policy ────────────────────────────────────────────
        try:
            from stable_baselines3 import PPO, SAC
            from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
        except ImportError:
            self.get_logger().error("stable-baselines3 not installed")
            sys.exit(1)

        # Auto-detect algorithm from model path or try both
        algo = "PPO"
        if "sac_" in str(model_path).lower():
            algo = "SAC"
        AlgoClass = PPO if algo == "PPO" else SAC

        self.get_logger().info(f"Loading {algo} {stage} model: {model_path}")
        self.model = AlgoClass.load(model_path)
        self._use_vecnorm = False

        norm_path = model_path.replace(".zip", "_vecnormalize.pkl")
        if os.path.exists(norm_path) and algo == "PPO":
            self._make_dummy = {
                "approach": _make_dummy_reach,
                "grasp": _make_dummy_grasp,
                "pick": _make_dummy_grasp,
            }[stage]
            dummy_env = DummyVecEnv([lambda: self._make_dummy()])
            self.vec_normalize = VecNormalize.load(norm_path, dummy_env)
            self._use_vecnorm = True
            self.get_logger().info(f"Loaded normalization from: {norm_path}")
        elif not os.path.exists(norm_path):
            self.get_logger().info("No VecNormalize stats found (SAC or first run)")

        # ── joint state tracking ───────────────────────────────────
        self._current_joints = {
            "joint_lift": 0.0,
            "joint_arm_l0": 0.0, "joint_arm_l1": 0.0,
            "joint_arm_l2": 0.0, "joint_arm_l3": 0.0,
            "joint_wrist_yaw": 0.0, "joint_gripper_slide": 0.04,
        }

        # ── ROS 2 interfaces ───────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/stretch/joint_states", self._joint_callback, 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, "/stretch/joint_command", 10)
        self.create_timer(step_delay, self._control_loop)

        self.get_logger().info(f"RL {stage} node started (target: {tomato_name})")

    # ── ROS callback ───────────────────────────────────────────────
    def _joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._current_joints and i < len(msg.position):
                self._current_joints[name] = msg.position[i]

    # ── FK helpers ─────────────────────────────────────────────────
    def _get_ctrl_state(self):
        """Reads joint positions from ROS joint states."""
        lift = self._current_joints.get("joint_lift", 0.0)
        arm = sum(self._current_joints.get(f"joint_arm_l{i}", 0.0) for i in range(4))
        wrist = self._current_joints.get("joint_wrist_yaw", 0.0)
        grip = self._current_joints.get("joint_gripper_slide", 0.04)
        return np.array([lift, arm, wrist, grip], dtype=np.float32)

    def _compute_ee(self):
        """Compute end-effector world position via FK.

        CRITICAL: sets the base qpos to the expected post-navigation position
        (anchor G), otherwise FK gives wrong world coordinates and dist-to-target
        checks in the success condition never pass.
        """
        ctrl = self._get_ctrl_state()
        # Set base position: anchor G is at (0.5, 3.75, 0), robot faces west (yaw=π)
        self.mj_data.qpos[0] = 0.5
        self.mj_data.qpos[1] = 3.75
        self.mj_data.qpos[2] = 0.0
        self.mj_data.qpos[3] = 0.0   # qw = cos(π/2) = 0
        self.mj_data.qpos[4] = 0.0
        self.mj_data.qpos[5] = 0.0
        self.mj_data.qpos[6] = 1.0   # qz = sin(π/2) = 1  (yaw = π)

        # Set arm joints
        self.mj_data.qpos[LIFT_QPOS] = ctrl[0]
        seg = ctrl[1] / 4.0
        for i in range(4):
            self.mj_data.qpos[ARM0_QPOS + i] = seg
        self.mj_data.qpos[WRIST_QPOS] = ctrl[2]
        self.mj_data.qpos[GRIP_QPOS] = ctrl[3]
        mujoco.mj_forward(self.mj_model, self.mj_data)
        return self.mj_data.site_xpos[self.ee_site_id].copy()

    # ── observation builders ───────────────────────────────────────
    def _build_obs_approach(self, joints, ee, dist):
        return np.array([
            joints[0], joints[1], joints[2],
            ee[0], ee[1], ee[2],
            self._tomato_pos[0], self._tomato_pos[1], self._tomato_pos[2],
            dist,
        ], dtype=np.float32)

    def _build_obs_grasp(self, joints, ee, dist):
        return np.array([
            joints[0], joints[1], joints[2], joints[3],
            ee[0], ee[1], ee[2],
            self._tomato_pos[0], self._tomato_pos[1], self._tomato_pos[2],
            dist,
        ], dtype=np.float32)

    # ── control loop ───────────────────────────────────────────────
    def _control_loop(self):
        if self._done:
            return

        self._step_count += 1
        if self._step_count > self._max_steps:
            self.get_logger().warn(f"Max steps ({self._max_steps}) reached")
            self._done = True
            return

        joints = self._get_ctrl_state()
        ee = self._compute_ee()
        dist = float(np.linalg.norm(ee - self._tomato_pos))

        # Build observation and predict
        if self._stage == "approach":
            obs = self._build_obs_approach(joints, ee, dist)
        else:
            obs = self._build_obs_grasp(joints, ee, dist)

        if self._use_vecnorm:
            obs = self.vec_normalize.normalize_obs(obs)

        action, _states = self.model.predict(obs, deterministic=True)

        # Publish joint commands
        if self._stage == "approach":
            self._publish_joint("lift", float(np.clip(joints[0] + action[0] * 0.06, -0.5, 0.6)))
            self._publish_joint("arm_extend", float(np.clip(joints[1] + action[1] * 0.08, 0.0, 0.52)))
            self._publish_joint("wrist_yaw", float(np.clip(joints[2] + action[2] * 0.12, -1.75, 4.0)))
        else:
            self._publish_joint("lift", float(np.clip(joints[0] + action[0] * 0.04, -0.5, 0.6)))
            self._publish_joint("arm_extend", float(np.clip(joints[1] + action[1] * 0.05, 0.0, 0.52)))
            self._publish_joint("wrist_yaw", float(np.clip(joints[2] + action[2] * 0.08, -1.75, 4.0)))
            self._publish_joint("gripper", float(np.clip(joints[3] + action[3] * 0.004, -0.005, 0.04)))

        # Log and check success
        if self._step_count % 20 == 0:
            self.get_logger().info(
                f"Step {self._step_count}/{self._max_steps} "
                f"dist={dist:.3f}m grip={joints[3]:.3f} lift={joints[0]:.3f}"
            )

        # Stage 1 success: EE within 10 cm of target
        if self._stage == "approach" and dist < 0.10:
            self.get_logger().info(f"✓ Approach complete — dist={dist*100:.1f}cm")
            self._done = True
            self._success = True

        # Stage 2 success: gripper closed AND arm raised
        elif self._stage == "grasp":
            if joints[3] < 0.01 and joints[0] > 0.15:
                self.get_logger().info(
                    f"✓ Grasp+lift complete — grip={joints[3]:.3f} lift={joints[0]:.3f}"
                )
                self._done = True
                self._success = True
            elif self._step_count % 20 == 0:
                # Show why not succeeded yet
                reasons = []
                if joints[3] >= 0.01:
                    reasons.append(f"gripper too open ({joints[3]:.3f} >= 0.01)")
                if joints[0] <= 0.15:
                    reasons.append(f"lift too low ({joints[0]:.3f} <= 0.15)")
                self.get_logger().info(f"  Not yet: {'; '.join(reasons)}")

    def _publish_joint(self, joint_name: str, value: float):
        if joint_name not in JOINT_ORDER:
            return
        msg = Float64MultiArray()
        msg.data = [float(JOINT_ORDER.index(joint_name)), value, 50.0]
        self.joint_pub.publish(msg)

    @property
    def success(self):
        return self._success

    @property
    def done(self):
        return self._done


def _make_dummy_reach():
    from stretch_sim.rl_env import StretchReachEnv
    return StretchReachEnv(max_episode_steps=10)


def _make_dummy_grasp():
    from stretch_sim.rl_env import StretchGraspEnv
    return StretchGraspEnv(max_episode_steps=10)


def main(args=None):
    parser = argparse.ArgumentParser(description="RL arm inference node")
    parser.add_argument("--stage", type=str, default="pick",
                        choices=["approach", "grasp", "pick"],
                        help="Inference stage (default: pick)")
    parser.add_argument("--target", type=str, default="tomato1",
                        choices=["tomato1", "tomato2", "tomato3"])
    parser.add_argument("--max-steps", type=int, default=200)
    parser.add_argument("--model", type=str, default=None)
    parser.add_argument("--step-delay", type=float, default=0.05)
    parsed = parser.parse_args(args)

    if parsed.model is None:
        repo_root = Path(__file__).resolve().parents[1]
        # Map "pick" back to the original single-stage model name
        stage_tag = "rl_arm" if parsed.stage == "pick" else f"rl_{parsed.stage}"
        parsed.model = str(repo_root / "models" / f"{stage_tag}_{parsed.target}.zip")

    if not os.path.exists(parsed.model):
        print(f"Error: Model not found: {parsed.model}")
        print(f"Train: python scripts/train_rl_arm.py --stage {parsed.stage} "
              f"--target {parsed.target}")
        sys.exit(1)

    rclpy.init(args=args)
    node = RLInferenceNode(
        model_path=parsed.model, stage=parsed.stage,
        tomato_name=parsed.target, max_steps=parsed.max_steps,
        step_delay=parsed.step_delay,
    )

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.success:
        print(f"✓ RL {parsed.stage} completed successfully")
    else:
        print(f"✗ RL {parsed.stage} failed")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    sys.exit(main())
