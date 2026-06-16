#!/usr/bin/env python3
"""ROS 2 node for Stretch 3 MuJoCo simulation (IK sample)."""

import sys
from pathlib import Path

_src = Path(__file__).resolve().parents[1] / "src"
if _src.is_dir() and str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

import os
import time
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray, String, Bool
import mujoco
import mujoco.viewer
import numpy as np
import cv2
from stretch_sim.anchor_utils import load_anchors_from_xml
from stretch_sim.controls import ArmExtendPID, LiftPID, WristYawPID
from stretch_sim.ik import IKSolver
from stretch_sim.navigation import NavigationController
from stretch_sim.paths import get_xml_path

# Joint configuration
JOINT_NAMES = [
    'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt'
]

# qpos indices must match the MuJoCo model jnt_qposadr values
JOINT_QPOS_MAP = {
    'joint_lift': 9, 'joint_arm_l3': 10, 'joint_arm_l2': 11,
    'joint_arm_l1': 12, 'joint_arm_l0': 13
}

JOINT_LIMITS = {
    'lift': (-0.5, 0.6), 'arm_extend': (0.0, 0.52), 'wrist_yaw': (-1.75, 4.0),
    'gripper': (-0.005, 0.04), 'head_pan': (-3.9, 1.5), 'head_tilt': (-1.53, 0.79)
}

ACTUATOR_NAMES = ['forward', 'turn', 'lift', 'arm_extend', 'wrist_yaw', 
                  'grip', 'head_pan', 'head_tilt']

JOINT_COMMAND_MAP = [
    ('lift', 'lift'), ('arm_extend', 'arm_extend'), ('wrist_yaw', 'wrist_yaw'),
    ('gripper', 'grip'), ('head_pan', 'head_pan'), ('head_tilt', 'head_tilt')
]

RESET_POSITIONS = {
    'lift': 0.6,
    'arm_extend': 0.0,
    'wrist_yaw': (4.0 + -1.75) / 2,  # Middle position (1.125)
    'gripper': 0.04
}

# Timing constants
PUB_RATE = 30.0  # Hz
RENDER_RATE = 20.0  # Hz
RESET_SPEED = 0.02
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
DEFAULT_SPEED = 50.0
JOINT_TOLERANCE = 0.001
ZERO_PLACEHOLDER_THRESHOLD = 0.05


class StretchSimNode(Node):
    """ROS 2 node that runs MuJoCo simulation and handles ROS 2 communication."""
    
    def __init__(self):
        super().__init__('stretch_sim')
        
        xml_path = str(get_xml_path())
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        self.anchors = self._load_anchors(xml_path)
        self._init_robot_state()
        self._init_actuators()
        self._init_camera()
        self._setup_ros2()
        self.ik=IKSolver(self.model,self.data,logger=self.get_logger())
        self.nav_controller = NavigationController()
        self.manual_control = False
        self.running = True
        self.sim_thread = None
        self._nav_log_counter = 0
        self._resetting_arm = False
        self._reset_targets = {}
        self._joint_targets = {}
        self._joint_speed_percent = {}
        self._base_joint_speed = 0.02
        self.joint_info = self.get_joint_indices_complete()
        self.get_logger().info('Stretch 2 ROS 2 Simulation Node started')
    
    def _load_anchors(self, xml_path):
        """Load anchors from XML file."""
        try:
            anchors = load_anchors_from_xml(xml_path)
            self.get_logger().info(f'Loaded {len(anchors)} anchors: {sorted(anchors.keys())}')
            for letter, data in sorted(anchors.items()):
                self.get_logger().info(f'  {letter}: {data}')
            return anchors
        except Exception as e:
            self.get_logger().error(f'Failed to load anchors: {e}')
            return {}
    
    def _init_robot_state(self):
        """Initialize robot to default state."""
        self.data.qpos[7] = 0.0  # Lift at bottom
        self.data.qpos[8:12] = 0.0  # Arm retracted
        mujoco.mj_forward(self.model, self.data)
    
    def _init_actuators(self):
        """Initialize actuator and joint mappings."""
        self.actuator_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        }
        self.ctrl_state = {name: 0.0 for name in ACTUATOR_NAMES}
        
        self.joint_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        }
        
        self.base_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
    
    def _init_camera(self):
        """Initialize camera if available."""
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'camera_rgb')
        if self.camera_id < 0:
            self.get_logger().warn('Camera "camera_rgb" not found, camera display disabled')
            self.camera_id = None
            self.camera_renderer = None
            self.camera_obj = None
        else:
            self.camera_renderer = None
            self.camera_obj = None
            self.get_logger().info(f'Camera "camera_rgb" found (ID: {self.camera_id})')
    
    def _setup_ros2(self):
        """Setup ROS 2 publishers and subscribers."""
        self.create_subscription(Twist, '/stretch/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/stretch/joint_command', 
                                self._joint_command_callback, 10)
        self.create_subscription(Float64MultiArray, '/stretch/joint_commands', 
                                self._joint_commands_callback, 10)  # Legacy
        self.create_subscription(String, '/stretch/navigate_to_anchor', 
                                self._navigate_to_anchor_callback, 10)
        self.create_subscription(String, '/stretch/turn_towards_anchor', 
                                self._turn_towards_anchor_callback, 10)
        self.create_subscription(String, '/stretch/reset_arm', 
                                self._reset_arm_callback, 10)
        self.create_subscription(Float64MultiArray, '/stretch/navigate_to_position', 
                                self._navigate_to_position_callback, 10)
        self.create_subscription(String, '/stretch/macro_action', 
                            self._macro_pick_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, '/stretch/joint_states', 10)
        self.nav_status_pub = self.create_publisher(Bool, '/stretch/navigation_active', 10)
        self.camera_pub = self.create_publisher(Image, '/stretch/camera/image_raw', 10)
    
    @staticmethod
    def _clamp_speed(speed_percent):
        """Clamp speed percentage to valid range."""
        return max(0.0, min(100.0, float(speed_percent)))
    
    def _set_joint_target(self, actuator_name, value, speed_percent):
        """Set target for a joint actuator."""
        if actuator_name not in self.ctrl_state:
            return False
        
        # Find corresponding cmd_name for limits
        cmd_name = next((cmd for cmd, act in JOINT_COMMAND_MAP if act == actuator_name), None)
        if cmd_name:
            min_val, max_val = JOINT_LIMITS[cmd_name]
            value = np.clip(value, min_val, max_val)
        
        self._joint_targets[actuator_name] = value
        self._joint_speed_percent[actuator_name] = speed_percent
        return True
    def _macro_pick_callback(self, msg):
        """
        Handle macro pick command. 
        
        Format: "tomato1" or "tomato1:G" or "tomato2:D"
        
        Examples:
            - "tomato1" → Uses default anchor G
            - "tomato2:G" → Pick tomato2, navigate via anchor G
            - "tomato3:D" → Pick tomato3, navigate via anchor D
        """
        data = msg.data.strip().lower()
        
        # Parse tomato name and optional anchor
        if ':' in data:
            parts = data.split(':')
            tomato_name = parts[0].strip()
            anchor_name = parts[1].strip().upper()
        else:
            tomato_name = data.strip()
            anchor_name = 'G'  # Default anchor
        
        # Validate tomato name
        valid_tomatoes = ['tomato1', 'tomato2', 'tomato3']
        if tomato_name not in valid_tomatoes:
            self.get_logger().error(f'Invalid tomato name: {tomato_name}. Valid: {valid_tomatoes}')
            return
        
        # Validate anchor exists
        if anchor_name not in self.anchors:
            available = ', '.join(sorted(self.anchors.keys()))
            self.get_logger().error(f'Unknown anchor: {anchor_name}. Available: {available}')
            return
        
        self.get_logger().info(f'🍅 MACRO: Pick {tomato_name} via anchor {anchor_name}')
        
        # Reset all state flags
        for attr in ['_alignment_checked', '_ik_computed', '_stored_q_sol', 
                    '_pick_stage', '_motion_stage', '_pick_timer']:
            if hasattr(self, attr):
                delattr(self, attr)
        
        # Store target tomato for the pipeline
        self._target_tomato = tomato_name
        self._target_anchor = anchor_name
        
        # Start navigation to anchor
        self._handle_anchor_command(anchor_name, turn_only=False, position_tolerance=None)
    def _joint_command_callback(self, msg):
        """Handle single joint position command: [joint_index, value, speed_percent]."""
        if self._resetting_arm or len(msg.data) < 2:
            return
        
        joint_index = int(msg.data[0])
        value = float(msg.data[1])
        speed_percent = self._clamp_speed(msg.data[2] if len(msg.data) > 2 else DEFAULT_SPEED)
        
        if not (0 <= joint_index < len(JOINT_COMMAND_MAP)):
            self.get_logger().warn(f'Invalid joint index: {joint_index}')
            return
        
        _, actuator_name = JOINT_COMMAND_MAP[joint_index]
        self._set_joint_target(actuator_name, value, speed_percent)
    
    def _joint_commands_callback(self, msg):
        """Handle multiple joint commands (legacy): [lift, arm_extend, ..., speed_percent]."""
        if self._resetting_arm or len(msg.data) < len(JOINT_COMMAND_MAP):
            return
        
        speed_percent = self._clamp_speed(
            msg.data[-1] if len(msg.data) > len(JOINT_COMMAND_MAP) else DEFAULT_SPEED
        )
        
        for i, ((cmd_name, actuator_name), value) in enumerate(
            zip(JOINT_COMMAND_MAP, msg.data[:len(JOINT_COMMAND_MAP)])
        ):
            if actuator_name not in self.ctrl_state:
                continue
            
            min_val, max_val = JOINT_LIMITS[cmd_name]
            target_value = np.clip(value, min_val, max_val)
            current_value = self.ctrl_state[actuator_name]
            
            # Skip zero placeholders for joints with negative minimum
            if (target_value == 0.0 and abs(current_value) > ZERO_PLACEHOLDER_THRESHOLD 
                and min_val < 0.0):
                continue
            
            self._set_joint_target(actuator_name, target_value, speed_percent)
    
    def _reset_arm_callback(self, msg):
        """Handle arm reset command: "reset" or "reset:speed_percent"."""
        data = msg.data.strip()
        if not data.lower().startswith('reset'):
            return
        
        speed_percent = DEFAULT_SPEED
        if ':' in data:
            try:
                speed_percent = self._clamp_speed(data.split(':', 1)[1])
            except ValueError:
                self.get_logger().warn(f'Invalid speed in reset command: {data}')
        
        self.get_logger().info(f'Starting arm reset (speed: {speed_percent}%)')
        self._resetting_arm = True
        self._reset_speed_percent = speed_percent
        self._reset_targets = {
            'lift': RESET_POSITIONS['lift'],
            'arm_extend': RESET_POSITIONS['arm_extend'],
            'wrist_yaw': RESET_POSITIONS['wrist_yaw'],
            'grip': RESET_POSITIONS['gripper']
        }
    
    def _update_arm_reset(self):
        """Smoothly move arm to reset positions."""
        if not self._resetting_arm:
            return
        
        speed_multiplier = getattr(self, '_reset_speed_percent', DEFAULT_SPEED) / DEFAULT_SPEED
        current_speed = RESET_SPEED * speed_multiplier
        all_reached = True
        
        for actuator_name, target in self._reset_targets.items():
            if actuator_name not in self.ctrl_state:
                continue
            
            current = self.ctrl_state[actuator_name]
            diff = target - current
            
            if abs(diff) < current_speed:
                self.ctrl_state[actuator_name] = target
            else:
                step = current_speed if diff > 0 else -current_speed
                self.ctrl_state[actuator_name] = current + step
                all_reached = False
        
        if all_reached:
            self._resetting_arm = False
            self.get_logger().info('✓ Arm reset complete')
    
    def _update_joint_movements(self):
        """Gradually move joints towards their target positions."""
        if not self._joint_targets:
            return
        
        for actuator_name, target in list(self._joint_targets.items()):
            if actuator_name not in self.ctrl_state:
                continue
            
            current = self.ctrl_state[actuator_name]
            diff = target - current
            
            if abs(diff) < JOINT_TOLERANCE:
                self.ctrl_state[actuator_name] = target
                self._joint_targets.pop(actuator_name, None)
                self._joint_speed_percent.pop(actuator_name, None)
                continue
            
            speed_multiplier = self._joint_speed_percent.get(actuator_name, DEFAULT_SPEED) / DEFAULT_SPEED
            current_speed = self._base_joint_speed * speed_multiplier
            
            if abs(diff) < current_speed:
                self.ctrl_state[actuator_name] = target
                self._joint_targets.pop(actuator_name, None)
                self._joint_speed_percent.pop(actuator_name, None)
            else:
                step = current_speed if diff > 0 else -current_speed
                self.ctrl_state[actuator_name] = current + step
    
    def _handle_anchor_command(self, anchor_key, turn_only=False, delta_angle=None, 
                               position_tolerance=None, target_angle_degrees=None):
        """
        Handle anchor navigation command.
        
        Args:
            anchor_key: Anchor identifier (e.g., "ORIGIN", "A", "B") or None if target_angle_degrees is used
            turn_only: If True, only turn towards anchor/angle without moving
            delta_angle: Optional delta angle in degrees for turn-only mode (default: 5.0)
            position_tolerance: Optional position tolerance in meters (default: 0.15)
            target_angle_degrees: Optional absolute target angle in degrees (0-360) for turn_towards
        """
        if turn_only:
            if target_angle_degrees is not None:
                # Absolute angle mode
                self.nav_controller.set_turn_only_target(None, delta_angle, target_angle_degrees)
                delta_value = delta_angle if delta_angle is not None else 5.0
                self.get_logger().info(f'Turning to absolute angle {target_angle_degrees:.1f}° (delta_angle={delta_value:.1f}°)')
            else:
                # Position-based mode
                anchor_key = anchor_key.strip().upper()
                if anchor_key not in self.anchors:
                    available = ', '.join(sorted(self.anchors.keys())) if self.anchors else 'none'
                    self.get_logger().warn(f'Unknown anchor: {anchor_key}. Available: {available}')
                    return
                anchor_data = self.anchors[anchor_key]
                target_pos = anchor_data['pos']
                current_pos, _ = self._get_robot_pose()
                distance = np.linalg.norm(np.array(target_pos[:2]) - current_pos[:2])
                self.nav_controller.set_turn_only_target(target_pos, delta_angle)
                delta_value = delta_angle if delta_angle is not None else 5.0
                self.get_logger().info(f'Turning towards anchor {anchor_key} (distance: {distance:.2f}m, delta_angle={delta_value:.1f}°)')
        else:
            # go_to_anchor: no direction alignment
            anchor_key = anchor_key.strip().upper()
            if anchor_key not in self.anchors:
                available = ', '.join(sorted(self.anchors.keys())) if self.anchors else 'none'
                self.get_logger().warn(f'Unknown anchor: {anchor_key}. Available: {available}')
                return
            anchor_data = self.anchors[anchor_key]
            target_pos = anchor_data['pos']
            current_pos, _ = self._get_robot_pose()
            distance = np.linalg.norm(np.array(target_pos[:2]) - current_pos[:2])
            # go_to_anchor no longer does direction alignment
            #
            #target_direction = anchor_data.get('direction')
            self.nav_controller.set_target(target_pos, None, position_tolerance, None)
            #self.nav_controller.set_target(target_pos, target_direction, position_tolerance, None)
            pos_tol_info = f", pos_tol={position_tolerance:.2f}m" if position_tolerance else ""
            self.get_logger().info(f'Navigating to anchor {anchor_key} (distance: {distance:.2f}m{pos_tol_info})')
        
        self.manual_control = False
        #
        if hasattr(self, '_alignment_started'):
            delattr(self, '_alignment_started')
        if hasattr(self, '_ik_computed'):
            delattr(self, '_ik_computed')
        if hasattr(self, '_stored_q_sol'):
            delattr(self, '_stored_q_sol')
        #
    def _navigate_to_anchor_callback(self, msg):
        """Handle navigate to anchor command. Format: "ANCHOR" or "ANCHOR:pos_tol"."""
        data = msg.data.strip()
        position_tolerance = None
        
        if ':' in data:
            parts = data.split(':')
            anchor_key = parts[0].strip().upper()
            try:
                if len(parts) >= 2:
                    position_tolerance = float(parts[1].strip())
            except (ValueError, IndexError):
                self.get_logger().warn(f'Invalid position_tolerance in go_to_anchor command: {data}, using default')
        else:
            anchor_key = data.strip().upper()
        
        self._handle_anchor_command(anchor_key, turn_only=False, 
                                   position_tolerance=position_tolerance)
    
    def _turn_towards_anchor_callback(self, msg):
        """Handle turn towards anchor/angle command. Format: "ANCHOR:delta_angle" or "degrees:target_angle:delta_angle"."""
        data = msg.data.strip()
        delta_angle = None
        target_angle_degrees = None
        anchor_key = None
        
        if ':' in data:
            parts = data.split(':')
            first_part = parts[0].strip().upper()
            
            if first_part == "DEGREES" or first_part.replace('.', '').replace('-', '').isdigit():
                # Absolute angle mode: "degrees:target_angle:delta_angle" or "target_angle:delta_angle"
                try:
                    if first_part == "DEGREES":
                        if len(parts) >= 2:
                            target_angle_degrees = float(parts[1].strip())
                        if len(parts) >= 3:
                            delta_angle = float(parts[2].strip())
                    else:
                        # Assume format: "target_angle:delta_angle"
                        target_angle_degrees = float(parts[0].strip())
                        if len(parts) >= 2:
                            delta_angle = float(parts[1].strip())
                except (ValueError, IndexError):
                    self.get_logger().warn(f'Invalid angle values in turn_towards command: {data}, using defaults')
            else:
                # Position-based mode: "ANCHOR:delta_angle"
                anchor_key = first_part
                try:
                    if len(parts) >= 2:
                        delta_angle = float(parts[1].strip())
                except (ValueError, IndexError):
                    self.get_logger().warn(f'Invalid delta_angle in turn_towards command: {data}, using default')
        else:
            # Simple format: just anchor name
            anchor_key = data.strip().upper()
        
        self._handle_anchor_command(anchor_key, turn_only=True, delta_angle=delta_angle,
                                   target_angle_degrees=target_angle_degrees)
    
    def _navigate_to_position_callback(self, msg):
        """Handle navigate to position command: [x, y, direction?, speed_percent?]."""
        if len(msg.data) < 2:
            self.get_logger().warn('Position command requires at least x and y coordinates')
            return
        
        x, y = float(msg.data[0]), float(msg.data[1])
        target_pos = [x, y, 0.0]
        target_direction = float(msg.data[2]) if len(msg.data) > 2 else None
        speed_percent = self._clamp_speed(msg.data[3] if len(msg.data) > 3 else DEFAULT_SPEED)
        
        current_pos, _ = self._get_robot_pose()
        distance = np.linalg.norm(np.array(target_pos[:2]) - current_pos[:2])
        
        self.nav_controller.set_target(target_pos, target_direction)
        self.manual_control = False
        
        direction_info = f", direction={math.degrees(target_direction):.1f}°" if target_direction else ""
        speed_info = f", speed={speed_percent}%" if speed_percent != DEFAULT_SPEED else ""
        self.get_logger().info(f'Navigating to ({x}, {y}) (distance: {distance:.2f}m{direction_info}{speed_info})')
    
    def _cmd_vel_callback(self, msg):
        """Handle base velocity commands."""
        has_manual_input = abs(msg.linear.x) > 0.05 or abs(msg.angular.z) > 0.05
        
        if has_manual_input and self.nav_controller.is_active():
            self.manual_control = True
            self.nav_controller.cancel()
            self.get_logger().info('Manual control override')
        
        linear_x = 0.0 if abs(msg.linear.x) < 0.001 and abs(msg.angular.z) > 0.001 else msg.linear.x
        self.ctrl_state['forward'] = linear_x
        self.ctrl_state['turn'] = msg.angular.z
    
    def _get_robot_pose(self):
        """Get current robot position and orientation."""
        return self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy()
    
    def _log_navigation_status(self, pos, quat, linear_vel, angular_vel):
        """Log navigation status for debugging."""
        target = self.nav_controller.target_pos
        if target is None:
            return
        
        distance = np.linalg.norm(target - pos[:2])
        diff = target - pos[:2]
        desired_angle = np.arctan2(diff[1], diff[0])
        
        w, x, y, z = quat
        current_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        angle_error = np.arctan2(
            np.sin(desired_angle - current_yaw),
            np.cos(desired_angle - current_yaw)
        )
        angle_error_deg = np.degrees(abs(angle_error))
        aligned = angle_error_deg <= 10.0
        
        self.get_logger().info(
            f'Nav: dist={distance:.2f}m, lin={linear_vel:.2f} m/s, '
            f'ctrl={self.ctrl_state.get("forward", 0.0):.3f}, ang={angular_vel:.2f}, '
            f'err={angle_error_deg:.1f}°, aligned={aligned}'
        )
    
    def _update_navigation(self):
        """Update navigation controller and apply its commands."""
        nav_status = Bool()
        nav_status.data = self.nav_controller.is_active() and not self.manual_control
        self.nav_status_pub.publish(nav_status)
        #
        if hasattr(self, '_stored_q_sol'):
            self.simple_pick()
            return
        #
        if not self.nav_controller.is_active() or self.manual_control:
            return
        
        pos, quat = self._get_robot_pose()
        linear_vel, angular_vel = self.nav_controller.get_control(pos, quat)
        
        self._nav_log_counter += 1
        if self._nav_log_counter % (10 if abs(linear_vel) > 0.01 else 50) == 0:
            self._log_navigation_status(pos, quat, linear_vel, angular_vel)
        
        if abs(linear_vel) < 0.001 and abs(angular_vel) > 0.001:
            linear_vel = 0.0
        
        self.ctrl_state['forward'] = linear_vel
        self.ctrl_state['turn'] = angular_vel
        
        if self.nav_controller.has_reached():
            self.get_logger().info('✓ Reached target position!')
            self.ctrl_state['forward'] = 0.0
            self.ctrl_state['turn'] = 0.0
            #
            tomato_name = getattr(self, '_target_tomato', 'tomato1')
                #
            pos,quat=self._get_robot_pose()
            yaw_diff, current_yaw, desired = self.ik.align_with_target(pos=pos,quat=quat,tomato_name=tomato_name)
            self.get_logger().info(f'  current_yaw:  {np.degrees(current_yaw):.2f}° ({current_yaw:.4f} rad)')
            self.get_logger().info(f'  desired_yaw:  {np.degrees(desired):.2f}° ({desired:.4f} rad)')
            self.get_logger().info(f'  yaw_diff:     {np.degrees(yaw_diff):.2f}° ({yaw_diff:.4f} rad)')
            #if not hasattr(self, '_alignment_started'):

            
            if abs(yaw_diff) > 0.07:
                self.nav_controller.set_target(pos, target_direction=desired+1.527)
                #
                #self._alignment_started = True
                self.get_logger().info('🔄 Starting alignment rotation...')
                #return
                #
            #else:
                # Already aligned
             #   self.get_logger().info('✓ Already aligned!')
                #self._alignment_started = True
                #self.get_logger().info('succ')
            
            if abs(yaw_diff) < 0.07:
                self.get_logger().info('🎯 Computing IK...')
                # Compute IK only once
                if not hasattr(self, '_ik_computed'):
                    self.get_logger().info('🎯 Computing IK...')
                    
                    '''fruit_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'tomato2')
                    
                    if fruit_body_id >= 0:
                        fruit_pos = self.data.xpos[fruit_body_id].copy()
                        self.get_logger().info(f'Actual fruit position: {fruit_pos}')
                    else:
                        self.get_logger().error('Fruit body not found!')
                        fruit_pos = np.array([-1.0, 5.54, 1.05])'''
                    # Get the site instead of body
                    fruit_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'{tomato_name}_site')

                    if fruit_site_id >= 0:
                        fruit_pos = self.data.site_xpos[fruit_site_id].copy()
                        self.get_logger().info(f'Actual fruit site position: {fruit_pos}')
                    else:
                        self.get_logger().error('Fruit site not found!')
                        fruit_pos = np.array([-1.0, 5.54, 1.05])

                    bole, self._q_above=self.ik.compute_ik(target_pos=fruit_pos+np.array([0,0,0.2]), max_iter=300, tol=0.01)
                    bole2,self._q_grasp =self.ik.compute_ik(target_pos=fruit_pos,max_iter=300,tol=0.01)
            
                    
                    if bole and bole2:
                        self.get_logger().info(f'Q solution: {self._q_above}')
                        self._stored_q_sol = self._q_above
                        self._ik_computed = True
                        self._pick_stage = 'above'
                        self._pick_timer = 0
                        self.nav_controller.cancel()
    
                    else:
                        self.get_logger().error('✗ IK failed!')
                        self.nav_controller.cancel()

    
    def simple_pick(self):
        """Simple pick: above → open → down → close → up"""
        
        self._pick_timer = getattr(self, '_pick_timer', 0) + 1
        if self._pick_stage == 'above':
            self.ctrl_state['grip'] = 0.04
            self.apply_ik_solution(self._q_above)
            
            if self._pick_timer > 1000:  # ~2 seconds
                self.get_logger().info('✓ Above fruit, descending...')
                self._pick_stage = 'down'
                self._pick_timer = 0
                if hasattr(self, '_motion_stage'):
                    delattr(self, '_motion_stage')
        
        # Stage 2: Descend (gripper open)
        elif self._pick_stage == 'down':
            self.ctrl_state['grip'] = 0.04  # Open
            self.apply_ik_solution(self._q_grasp)
            
            if self._pick_timer > 1000:  # ~2 seconds
                self.get_logger().info('✓ At fruit, closing gripper...')
                self._pick_stage = 'close'
                self._pick_timer = 0
        
        # Stage 3: Close gripper
        elif self._pick_stage == 'close':
            self.ctrl_state['grip'] = 0.01  # Close
            self.apply_ik_solution(self._q_grasp)
            
            if self._pick_timer > 1000:  # ~1 second
                self.get_logger().info('✓ Grasped, lifting...')
                self._pick_stage = 'up'
                self._pick_timer = 0
                if hasattr(self, '_motion_stage'):
                    delattr(self, '_motion_stage')
        
        # Stage 4: Lift up
        elif self._pick_stage == 'up':
            self.ctrl_state['grip'] = 0.01  # Closed
            self.apply_ik_solution(self._q_grasp)
            self.apply_ik_solution(self._q_above)
            
            if self._pick_timer > 1000:  # ~2 seconds
                self.get_logger().info('🎉 Pick complete!')
                self._pick_stage = 'done'
        
        # Stage 5: Hold
        elif self._pick_stage == 'done':
            self.ctrl_state['grip'] = 0.01  # Closed
            self.apply_ik_solution(np.array([self._q_above[0],0,0,0,0,self._q_above[5]]))
        #
    def get_joint_indices_complete(self):
        """Get both qpos and nv (dof) indices for each joint."""
        
        joint_info = {}
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("JOINT INDEXING (qpos and nv)")
        self.get_logger().info("="*70)
        
        for joint_name in JOINT_NAMES:
            if joint_name in self.joint_ids:
                joint_id = self.joint_ids[joint_name]
                qpos_addr = self.model.jnt_qposadr[joint_id]
                
                # nv index (velocity space) - THIS IS WHAT WE NEED FOR JACOBIAN
                nv_addr = self.model.jnt_dofadr[joint_id]
                
                joint_info[joint_name] = {
                    'joint_id': joint_id,
                    'qpos_idx': qpos_addr,
                    'nv_idx': nv_addr
                }
                
                self.get_logger().info(
                    f"  {joint_name:<20} joint_id={joint_id:<3} "
                    f"qpos_idx={qpos_addr:<3} nv_idx={nv_addr:<3}"
                )
        
        self.get_logger().info("="*70 + "\n")
        
        return joint_info
    def get_joint_state(self, joint_name):

        if hasattr(self, 'joint_info') and joint_name in self.joint_info:
            qpos_idx = self.joint_info[joint_name]['qpos_idx']
            nv_idx = self.joint_info[joint_name]['nv_idx']
            
            if 0 <= qpos_idx < len(self.data.qpos) and 0 <= nv_idx < len(self.data.qvel):
                pos = float(self.data.qpos[qpos_idx])
                vel = float(self.data.qvel[nv_idx])
                return (pos, vel)
            
    def apply_ik_solution(self, q_sol,pick=False):

        if not hasattr(self, '_motion_stage'):
            self._motion_stage = 'lift'
            self._lift_kp = 2.3  # Proportional gain for lift control
            self.lift_ctr=LiftPID(Kp=150.0, Ki=50.0, Kd=20.0)
            self.arm_ctr = ArmExtendPID(Kp=200, Ki=5, Kd=10)
            self.wrist_ctr = WristYawPID(Kp=100.0, Ki=10.0, Kd=8.0)
        if self._motion_stage == 'lift':
            current_lift, current_vel = self.get_joint_state('joint_lift')
            target_lift = q_sol[0]
            lift_err=target_lift-current_lift
            dt = float(self.model.opt.timestep)
            force=self.lift_ctr.compute(z_desired=target_lift, z_current=current_lift, zd_current=current_vel, dt=dt)
            self.ctrl_state['lift'] =target_lift#np.clip(force,-56,56)
            self.get_logger().info(
                f"LIFT: current={current_lift:.3f} → target={target_lift:.3f}, "
                f"error={lift_err:.3f}"
            )
            self.ctrl_state['arm_extend'] = 0.0
            self.ctrl_state['wrist_yaw'] = 0.0
            if lift_err < 0.01:
                self.get_logger().info("✓ Lift complete! Moving to arm/wrist stage")
                self._motion_stage = 'arm'
            return
        
        if self._motion_stage == 'arm':
            target_lift = float(q_sol[0])
            self.ctrl_state['lift'] = target_lift
            self.ctrl_state['wrist_yaw'] = 0.0
            current_arm_total = self.data.qpos[10:14]
            target_arm_total = q_sol[1:5]
            current_arm_vel = self.data.qvel[9:13]
            dt = float(self.model.opt.timestep)
            self.arm_tau = self.arm_ctr.compute(
                x_desired=target_arm_total,
                x_current=current_arm_total,
                xd_current=current_arm_vel,
                dt=dt
            )
            self.ctrl_state['arm_extend'] = sum(target_arm_total)#float(self.arm_tau)
            arm_error = abs(sum(target_arm_total) - sum(current_arm_total))
            self.get_logger().info(
                f"ARM: {sum(current_arm_total):.4f} → {sum(target_arm_total):.4f}, "
                f"error={arm_error*1000:.1f}mm"
            )
            if arm_error < 0.002: 
                self.get_logger().info("✓ Arm extended! Moving to grasp stage")
                self._motion_stage = 'wrist'
            return
        if self._motion_stage == 'wrist':
                self.ctrl_state['lift'] =q_sol[0]
                self.ctrl_state['arm_extend'] = sum(q_sol[1:5])
                current_wrist, wrist_vel = self.get_joint_state('joint_wrist_yaw')
                target_wrist = float(q_sol[5])
                dt = float(self.model.opt.timestep)
                wrist_tau = self.wrist_ctr.compute(
                    target=target_wrist,
                    current=current_wrist,
                    velocity=wrist_vel,
                    dt=dt
                )
                self.ctrl_state['wrist_yaw'] = target_wrist#float(wrist_tau)
                wrist_error = abs(target_wrist - current_wrist)
                self.get_logger().info(
                    f"WRIST: {np.degrees(current_wrist):.1f}° → {np.degrees(target_wrist):.1f}°, "
                    f"error={np.degrees(wrist_error):.1f}°, tau={wrist_tau:.2f}N·m"
                )
                if wrist_error < 0.005:  # ~3 degrees tolerance
                    self.get_logger().info("✓ Wrist aligned! Moving to arm stage")
                    #self._motion_stage = 'grasp'
                return
        '''if self._motion_stage == 'grasp':
            target_lift = float(q_sol[0])
            self.ctrl_state['lift'] = target_lift
            target_wrist = float(q_sol[5])
            self.ctrl_state['wrist_yaw'] = target_wrist
            self.ctrl_state['arm_extend'] = sum(q_sol[1:5])
            self.get_logger().info("Grasping fruit...")
            if pick:
                self.ctrl_state['grip'] = 0.04
            else:
                self.ctrl_state['grip'] = -0.005
            return'''
    #
    #eges
    '''def _get_joint_state(self, joint_name):
        """Get position and velocity for a joint."""
        if joint_name in JOINT_QPOS_MAP:
            idx = JOINT_QPOS_MAP[joint_name]
            if idx < len(self.data.qpos):
                return (
                    float(self.data.qpos[idx]),
                    float(self.data.qvel[idx]) if idx < len(self.data.qvel) else 0.0
                )
        
        if joint_name in self.joint_ids:
            try:
                joint_id = self.joint_ids[joint_name]
                qpos_addr = self.model.jnt_qposadr[joint_id]
                qvel_addr = self.model.jnt_dofadr[joint_id]
                pos = self.data.qpos[qpos_addr] if 0 <= qpos_addr < len(self.data.qpos) else 0.0
                vel = self.data.qvel[qvel_addr] if 0 <= qvel_addr < len(self.data.qvel) else 0.0
                return (float(pos), float(vel))
            except (AttributeError, IndexError):
                pass
        
        return (0.0, 0.0)'''
    #eges
    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for joint_name in JOINT_NAMES:
            pos, vel = self.get_joint_state(joint_name)
            msg.name.append(joint_name)
            msg.position.append(pos)
            msg.velocity.append(vel)
            msg.effort.append(0.0)
        
        self.joint_state_pub.publish(msg)
    
    def _render_camera(self):
        """Render camera view and publish it."""
        if self.camera_id is None or self.camera_renderer is None:
            return
        
        try:
            self.camera_obj.fixedcamid = self.camera_id
            self.camera_obj.type = mujoco.mjtCamera.mjCAMERA_FIXED
            
            self.camera_renderer.update_scene(self.data, camera=self.camera_obj)
            camera_rgb = self.camera_renderer.render()
            camera_bgr = cv2.cvtColor(camera_rgb, cv2.COLOR_RGB2BGR)
            camera_bgr_rotated = cv2.rotate(camera_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            cv2.imshow('Robot Camera', camera_bgr_rotated)
            cv2.waitKey(1)
            
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_rgb'
            img_msg.height = CAMERA_HEIGHT
            img_msg.width = CAMERA_WIDTH
            img_msg.encoding = 'rgb8'
            img_msg.is_bigendian = False
            img_msg.step = CAMERA_WIDTH * 3
            img_msg.data = camera_rgb.tobytes()
            self.camera_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Camera rendering error: {e}')
    
    def _init_camera_rendering(self):
        """Initialize camera rendering components."""
        if self.camera_id is None:
            return
        
        try:
            self.camera_renderer = mujoco.Renderer(
                self.model, height=CAMERA_HEIGHT, width=CAMERA_WIDTH
            )
            self.camera_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
            self.camera_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = True
            self.camera_obj = mujoco.MjvCamera()
            self.camera_obj.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.camera_obj.fixedcamid = self.camera_id
            cv2.namedWindow('Robot Camera', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Robot Camera', CAMERA_HEIGHT, CAMERA_WIDTH)
            self.get_logger().info('Camera rendering initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera rendering: {e}')
            self.camera_id = None
    
    def run_simulation(self):
        """Run the MuJoCo simulation loop."""
        with mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            viewer.cam.lookat[:] = [0, 3, 1]
            viewer.cam.distance = 5
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 90
            
            self._init_camera_rendering()
            
            prev_render_time = prev_pub_time = prev_camera_time = time.time()
            pub_interval = 1.0 / PUB_RATE
            render_interval = 1.0 / RENDER_RATE
            camera_interval = 1.0 / PUB_RATE
            
            while self.running and viewer.is_running():
                step_start = time.time()
                
                self._update_navigation()
                self._update_arm_reset()
                self._update_joint_movements()
                
                for name, actuator_id in self.actuator_ids.items():
                    self.data.ctrl[actuator_id] = self.ctrl_state[name]
                
                mujoco.mj_step(self.model, self.data)
                
                now = time.time()
                if now - prev_pub_time > pub_interval:
                    self.publish_joint_states()
                    prev_pub_time = now
                
                if now - prev_camera_time > camera_interval:
                    self._render_camera()
                    prev_camera_time = now
                
                if now - prev_render_time > render_interval:
                    viewer.sync()
                    prev_render_time = now
                
                elapsed = time.time() - step_start
                if elapsed < self.model.opt.timestep:
                    time.sleep(self.model.opt.timestep - elapsed)
            
            if self.camera_id is not None:
                cv2.destroyAllWindows()
    
    def start_simulation(self):
        """Start simulation in a separate thread."""
        self.sim_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.sim_thread.start()
    
    def stop(self):
        """Stop the simulation."""
        self.running = False
        if self.sim_thread:
            self.sim_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = StretchSimNode()
    node.start_simulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
