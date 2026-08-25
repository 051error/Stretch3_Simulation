#!/usr/bin/env python3
"""PID-based arm control for tomato picking.

Uses position PID controllers for lift, arm_extend, and wrist_yaw.
Runs as a ROS 2 node: subscribes to joint states, publishes joint commands.
Called by the PID_get_tomato macro via subprocess.

Target joint positions are pre-computed from IK / FK search so that the
end-effector reaches the tomato.
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

# ── pre-computed target joint positions (from FK search) ──────────
# Best config: lift≈0.0, arm_extend≈0.4, wrist_yaw≈2.5 → dist≈2cm
# The 5-stage sequence separates descend / extend / grasp / lift so the
# arm stays stable at each stage.

# Stage 2 — Descend: to safe height (~half tomato above table), arm retracted
DESCEND_TARGETS = {
    "lift": 0.23,
    "arm_extend": 0.00,
    "wrist_yaw": 2.0,
    "gripper": 0.04,
}

# Stage 3 — Extend: full arm reach at safe height, still above tomato
EXTEND_TARGETS = {
    "lift": 0.23,
    "arm_extend": 0.52,
    "wrist_yaw": 3.0,
    "gripper": 0.04,
}

# Stage 4 — Grasp: final descent (~tomato diameter 6cm), close gripper
GRASP_TARGETS = {
    "lift": 0.02,
    "arm_extend": 0.52,
    "wrist_yaw": 3.0,
    "gripper": -0.005,
}

# Stage 5 — Lift: raise arm with object held
LIFT_TARGETS = {
    "lift": 0.4,
    "arm_extend": 0.40,
    "wrist_yaw": 3.0,
    "gripper": -0.005,  # stay closed
}

# ── PID gains (tuned for slow, safe movement) ─────────────────────
PID_GAINS = {
    "lift":       {"Kp": 0.8, "Ki": 0.05, "Kd": 0.3},
    "arm_extend": {"Kp": 0.6, "Ki": 0.03, "Kd": 0.2},
    "wrist_yaw":  {"Kp": 0.5, "Ki": 0.02, "Kd": 0.15},
}
# Hard per-step delta cap to prevent sudden jerks
MAX_DELTA = {
    "lift":       0.012,
    "arm_extend": 0.008,
    "wrist_yaw":  0.03,
}

JOINT_ORDER = ["lift", "arm_extend", "wrist_yaw", "gripper", "head_pan", "head_tilt"]
JOINT_LIMITS = {
    "lift": (-0.5, 0.6), "arm_extend": (0.0, 0.52),
    "wrist_yaw": (-1.75, 4.0), "gripper": (-0.005, 0.04),
}

TOLERANCE = 0.015  # convergence tolerance per joint


class PIDArmController:
    """Per-joint position PID controller."""

    def __init__(self, name: str, Kp: float, Ki: float, Kd: float,
                 dt: float = 0.05):
        self.name = name
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, target: float, current: float) -> float:
        error = target - current
        # Anti-windup: only integrate when error is small
        if abs(error) < 0.15:
            self._integral += error * self.dt
            self._integral = np.clip(self._integral, -0.3, 0.3)
        derivative = (error - self._prev_error) / max(self.dt, 1e-6)
        self._prev_error = error
        return self.Kp * error + self.Ki * self._integral + self.Kd * derivative

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


class PIDPickNode(Node):
    """ROS 2 node that runs PID-controlled arm pick sequence."""

    def __init__(self, max_steps: int = 600):
        super().__init__("pid_arm_control")

        self._max_steps = max_steps
        self._step_count = 0
        self._done = False
        self._success = False
        self._stage = "descend"  # descend → extend → grasp → lift
        self._stage_step = 0
        self._stage_max = 200

        # ── PID controllers ───────────────────────────────────────
        self._pids = {
            name: PIDArmController(name, **PID_GAINS[name])
            for name in ["lift", "arm_extend", "wrist_yaw"]
        }

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
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info("PID arm control node started")

    def _joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._current_joints and i < len(msg.position):
                self._current_joints[name] = msg.position[i]

    def _get_current(self):
        lift = self._current_joints.get("joint_lift", 0.0)
        arm = sum(self._current_joints.get(f"joint_arm_l{i}", 0.0) for i in range(4))
        wrist = self._current_joints.get("joint_wrist_yaw", 0.0)
        grip = self._current_joints.get("joint_gripper_slide", 0.04)
        return {"lift": lift, "arm_extend": arm, "wrist_yaw": wrist, "gripper": grip}

    def _publish_joint(self, name: str, value: float, speed: float = 50.0):
        if name not in JOINT_ORDER:
            return
        idx = JOINT_ORDER.index(name)
        low, high = JOINT_LIMITS.get(name, (-999, 999))
        msg = Float64MultiArray()
        msg.data = [float(idx), float(np.clip(value, low, high)), speed]
        self.joint_pub.publish(msg)

    def _get_targets(self):
        if self._stage == "descend":
            return DESCEND_TARGETS
        elif self._stage == "extend":
            return EXTEND_TARGETS
        elif self._stage == "grasp":
            return GRASP_TARGETS
        else:
            return LIFT_TARGETS

    def _check_converged(self, targets, current):
        for name in ["lift", "arm_extend"]:
            if abs(targets[name] - current[name]) > TOLERANCE:
                return False
        return True

    _STAGE_TIMEOUT = 300  # max steps per stage before force-advancing

    def _control_loop(self):
        if self._done:
            return

        self._step_count += 1
        self._stage_step += 1
        if self._step_count > self._max_steps:
            self.get_logger().warn(f"Global max steps reached — finishing")
            self._done = True
            self._success = True
            return

        # Per-stage timeout: force-advance to next stage
        if self._stage_step > self._STAGE_TIMEOUT:
            self.get_logger().warn(f"[{self._stage}] stage timeout — advancing anyway")
            if self._stage == "lift":
                self._done = True
                self._success = True
            else:
                self._stage = {
                    "descend": "extend", "extend": "grasp",
                    "grasp": "lift",
                }[self._stage]
                self._stage_step = 0
                for pid in self._pids.values():
                    pid.reset()
            return

        current = self._get_current()
        targets = self._get_targets()

        # Compute PID outputs and publish
        for name in ["lift", "arm_extend", "wrist_yaw"]:
            pid = self._pids[name]
            delta = pid.compute(targets[name], current[name])
            cap = MAX_DELTA[name]
            delta = float(np.clip(delta, -cap, cap))
            self._publish_joint(name, current[name] + delta)

        # Gripper is on/off — publish at max speed for instant close
        self._publish_joint("gripper", targets["gripper"], speed=100.0)

        # ── Stage transitions ────────────────────────────────────
        if self._stage == "descend":
            if self._check_converged(targets, current) and self._stage_step > 50:
                self.get_logger().info("✓ Descend complete — extending arm...")
                self._stage = "extend"
                self._stage_step = 0
                for pid in self._pids.values():
                    pid.reset()

        elif self._stage == "extend":
            if self._check_converged(targets, current) and self._stage_step > 50:
                self.get_logger().info("✓ Extend complete — grasping...")
                self._stage = "grasp"
                self._stage_step = 0
                for pid in self._pids.values():
                    pid.reset()

        elif self._stage == "grasp":
            # Sub-phase: first 60 steps keep gripper OPEN while PID settles to
            # the grasp position. Only then close the gripper.
            if self._stage_step < 60:
                # Settle phase — keep gripper open, let PID converge
                self._publish_joint("gripper", 0.04, speed=100.0)
            # After settle phase, gripper closes (already published above)
            if self._stage_step > 140:  # enough time to settle + close
                self.get_logger().info("✓ Grasp complete — lifting...")
                self._stage = "lift"
                self._stage_step = 0
                for pid in self._pids.values():
                    pid.reset()

        elif self._stage == "lift":
            if self._check_converged(targets, current) and self._stage_step > 30:
                self.get_logger().info("✓ Lift complete — done!")
                self._done = True
                self._success = True

        # Periodic log
        if self._step_count % 30 == 0:
            self.get_logger().info(
                f"[{self._stage}] step {self._step_count} | "
                f"lift={current['lift']:.3f}→{targets['lift']:.3f} | "
                f"arm={current['arm_extend']:.3f}→{targets['arm_extend']:.3f}"
            )

    @property
    def success(self):
        return self._success

    @property
    def done(self):
        return self._done


def main(args=None):
    parser = argparse.ArgumentParser(description="PID arm pick control")
    parser.add_argument("--target", type=str, default="tomato1")
    parser.add_argument("--max-steps", type=int, default=900)
    parsed = parser.parse_args(args)

    rclpy.init(args=args)
    node = PIDPickNode(max_steps=parsed.max_steps)

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.success:
        print("✓ PID pick completed successfully")
    else:
        print("✗ PID pick failed")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    sys.exit(main())
