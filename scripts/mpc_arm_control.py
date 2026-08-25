#!/usr/bin/env python3
"""MPC (Model Predictive Control) arm control for tomato picking.

Uses random-sampling shooting MPC: at each step, sample K random action
sequences of horizon H, predict their outcomes with a simple kinematic model,
and apply the first action of the best sequence.

Runs as a ROS 2 node. Called by the MPC_get_tomato macro via subprocess.
"""

import argparse
import math
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

# ── pre-computed target joint positions ───────────────────────────
APPROACH_TARGETS = {"lift": 0.05, "arm_extend": 0.40, "wrist_yaw": 2.5, "gripper": 0.04}
GRASP_TARGETS   = {"lift": 0.0,  "arm_extend": 0.40, "wrist_yaw": 2.5, "gripper": -0.005}
LIFT_TARGETS    = {"lift": 0.4,  "arm_extend": 0.40, "wrist_yaw": 2.5, "gripper": -0.005}

JOINT_ORDER = ["lift", "arm_extend", "wrist_yaw", "gripper", "head_pan", "head_tilt"]
JOINT_LIMITS = {
    "lift": (-0.5, 0.6), "arm_extend": (0.0, 0.52),
    "wrist_yaw": (-1.75, 4.0), "gripper": (-0.005, 0.04),
}

# ── MPC parameters ────────────────────────────────────────────────
MPC_HORIZON = 6        # prediction horizon (steps)
MPC_SAMPLES = 200      # number of random action sequences sampled
MPC_DT = 0.05          # time step for the kinematic model
ACTION_SCALE = np.array([0.012, 0.008, 0.03], dtype=np.float32)  # max delta per step
CONVERGENCE_TOL = 0.015
LIFT_QPOS = 9; ARM0_QPOS = 10; WRIST_QPOS = 14; GRIP_QPOS = 15


class MPCPickNode(Node):
    """ROS 2 node that runs MPC arm control for picking."""

    def __init__(self, max_steps: int = 600):
        super().__init__("mpc_arm_control")
        self._max_steps = max_steps
        self._step_count = 0
        self._done = False
        self._success = False
        self._stage = "approach"
        self._stage_step = 0

        # ── load MuJoCo model for FK (cost computation) ───────────
        xml_path = str(get_xml_path())
        self._mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self._mj_data = mujoco.MjData(self._mj_model)
        self._ee_site_id = mujoco.mj_name2id(
            self._mj_model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        # Tomato position
        self._tomato_pos = np.array([0.43, 4.0, 0.65])

        # ── joint state tracking ──────────────────────────────────
        self._current_joints = {
            "joint_lift": 0.0,
            "joint_arm_l0": 0.0, "joint_arm_l1": 0.0,
            "joint_arm_l2": 0.0, "joint_arm_l3": 0.0,
            "joint_wrist_yaw": 0.0, "joint_gripper_slide": 0.04,
        }

        # ── ROS 2 interfaces ──────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/stretch/joint_states", self._joint_callback, 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, "/stretch/joint_command", 10)
        self.create_timer(MPC_DT, self._control_loop)

        self.get_logger().info("MPC arm control node started")

    def _joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._current_joints and i < len(msg.position):
                self._current_joints[name] = msg.position[i]

    def _get_state(self):
        lift = self._current_joints.get("joint_lift", 0.0)
        arm = sum(self._current_joints.get(f"joint_arm_l{i}", 0.0) for i in range(4))
        wrist = self._current_joints.get("joint_wrist_yaw", 0.0)
        grip = self._current_joints.get("joint_gripper_slide", 0.04)
        return np.array([lift, arm, wrist, grip], dtype=np.float32)

    def _ee_distance(self, state):
        """Compute EE distance to target for a given joint configuration."""
        self._mj_data.qpos[0:3] = [0.5, 3.75, 0.0]
        self._mj_data.qpos[3:7] = [0.0, 0.0, 0.0, 1.0]
        self._mj_data.qpos[LIFT_QPOS] = state[0]
        seg = state[1] / 4.0
        for i in range(4):
            self._mj_data.qpos[ARM0_QPOS + i] = seg
        self._mj_data.qpos[WRIST_QPOS] = state[2]
        self._mj_data.qpos[GRIP_QPOS] = state[3]
        mujoco.mj_forward(self._mj_model, self._mj_data)
        ee = self._mj_data.site_xpos[self._ee_site_id]
        return float(np.linalg.norm(ee - self._tomato_pos))

    def _predict(self, state, action_delta):
        """Simple kinematic model: state += action_delta, clamped."""
        new_state = state.copy()
        new_state[:3] += action_delta
        new_state[0] = np.clip(new_state[0], JOINT_LIMITS["lift"][0], JOINT_LIMITS["lift"][1])
        new_state[1] = np.clip(new_state[1], JOINT_LIMITS["arm_extend"][0], JOINT_LIMITS["arm_extend"][1])
        new_state[2] = np.clip(new_state[2], JOINT_LIMITS["wrist_yaw"][0], JOINT_LIMITS["wrist_yaw"][1])
        return new_state

    def _mpc_step(self, state, targets):
        """Random-shooting MPC: sample action sequences, pick best."""
        best_cost = float("inf")
        best_action = np.zeros(3, dtype=np.float32)

        n_actions = 3  # lift, arm, wrist
        for _ in range(MPC_SAMPLES):
            # Sample a random action sequence of length H
            actions = (np.random.rand(MPC_HORIZON, n_actions) * 2 - 1) * ACTION_SCALE
            # Predict trajectory
            s = state.copy()
            total_cost = 0.0
            for t in range(MPC_HORIZON):
                s = self._predict(s, actions[t])
                # Cost: weighted distance to targets + control effort
                joint_cost = (
                    5.0 * abs(s[0] - targets["lift"]) +
                    3.0 * abs(s[1] - targets["arm_extend"]) +
                    1.0 * abs(s[2] - targets["wrist_yaw"])
                )
                action_cost = 0.01 * np.sum(np.square(actions[t]))
                total_cost += joint_cost + action_cost
            if total_cost < best_cost:
                best_cost = total_cost
                best_action = actions[0]

        return best_action

    def _publish_joint(self, name: str, value: float, speed: float = 50.0):
        if name not in JOINT_ORDER:
            return
        idx = JOINT_ORDER.index(name)
        low, high = JOINT_LIMITS.get(name, (-999, 999))
        msg = Float64MultiArray()
        msg.data = [float(idx), float(np.clip(value, low, high)), speed]
        self.joint_pub.publish(msg)

    def _get_targets(self):
        if self._stage == "approach":
            return APPROACH_TARGETS
        elif self._stage == "grasp":
            return GRASP_TARGETS
        else:
            return LIFT_TARGETS

    def _control_loop(self):
        if self._done:
            return

        self._step_count += 1
        self._stage_step += 1
        if self._step_count > self._max_steps:
            self.get_logger().warn(f"Max steps reached")
            self._done = True
            return

        state = self._get_state()
        targets = self._get_targets()

        # Gripper: set directly (binary action)
        self._publish_joint("gripper", targets["gripper"], speed=100.0)

        # MPC for lift, arm, wrist
        action = self._mpc_step(state, targets)
        for i, name in enumerate(["lift", "arm_extend", "wrist_yaw"]):
            self._publish_joint(name, state[i] + action[i])

        # Stage transitions
        errors = [abs(targets[name] - state[i]) for i, name in
                  enumerate(["lift", "arm_extend", "wrist_yaw"])]
        converged = all(e < CONVERGENCE_TOL for e in errors)

        if self._stage == "approach":
            if converged and self._stage_step > 50:
                self.get_logger().info("✓ MPC approach done — grasping...")
                self._stage = "grasp"
                self._stage_step = 0

        elif self._stage == "grasp":
            if self._stage_step > 80:
                self.get_logger().info("✓ MPC grasp done — lifting...")
                self._stage = "lift"
                self._stage_step = 0

        elif self._stage == "lift":
            if converged and self._stage_step > 30:
                self.get_logger().info("✓ MPC lift done!")
                self._done = True
                self._success = True

        if self._step_count % 30 == 0:
            dist = self._ee_distance(state)
            self.get_logger().info(
                f"[{self._stage}] step {self._step_count} "
                f"dist={dist:.3f}m | "
                f"l={state[0]:.3f}→{targets['lift']:.3f} "
                f"a={state[1]:.3f}→{targets['arm_extend']:.3f}"
            )

    @property
    def success(self):
        return self._success

    @property
    def done(self):
        return self._done


def main(args=None):
    parser = argparse.ArgumentParser(description="MPC arm pick control")
    parser.add_argument("--target", type=str, default="tomato1")
    parser.add_argument("--max-steps", type=int, default=600)
    parsed = parser.parse_args(args)

    rclpy.init(args=args)
    node = MPCPickNode(max_steps=parsed.max_steps)

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.success:
        print("✓ MPC pick completed successfully")
    else:
        print("✗ MPC pick failed")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    sys.exit(main())
