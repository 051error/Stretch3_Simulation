#!/usr/bin/env python3
"""Test script to verify ROS 2 communication with Stretch simulation."""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def test_imports():
    """Test if ROS 2 packages can be imported."""
    print("=" * 60)
    print("Testing ROS 2 Imports")
    print("=" * 60)
    
    try:
        import rclpy
        from geometry_msgs.msg import Twist, JointState
        from std_msgs.msg import Float64MultiArray, String
        print("✓ All ROS 2 imports successful")
        return True
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        print("  Solution: Install ROS 2 and source setup.bash")
        return False


def test_topics():
    """Test if topics can be created."""
    print("\n" + "=" * 60)
    print("Testing Topic Creation")
    print("=" * 60)
    
    try:
        rclpy.init()
        node = Node('test_node')
        
        topics = [
            ('/stretch/cmd_vel', Twist),
            ('/stretch/joint_commands', Float64MultiArray),
            ('/stretch/navigate_to_anchor', String)
        ]
        
        for topic_name, msg_type in topics:
            node.create_publisher(msg_type, topic_name, 10)
            print(f"✓ Created publisher: {topic_name}")
        
        node.destroy_node()
        rclpy.shutdown()
        return True
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def send_test_commands():
    """Send test commands to verify communication."""
    print("\n" + "=" * 60)
    print("Sending Test Commands")
    print("=" * 60)
    
    try:
        rclpy.init()
        node = Node('test_controller')
        
        cmd_vel_pub = node.create_publisher(Twist, '/stretch/cmd_vel', 10)
        joint_pub = node.create_publisher(Float64MultiArray, '/stretch/joint_commands', 10)
        anchor_pub = node.create_publisher(String, '/stretch/navigate_to_anchor', 10)
        
        time.sleep(1.0)  # Wait for publishers to be ready
        
        # Create test messages
        msg_forward = Twist()
        msg_forward.linear.x = 0.5
        msg_stop = Twist()
        msg_joint = Float64MultiArray()
        msg_joint.data = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg_anchor = String()
        msg_anchor.data = "A"
        
        tests = [
            ("Base velocity (forward)", lambda: cmd_vel_pub.publish(msg_forward)),
            ("Stop", lambda: cmd_vel_pub.publish(msg_stop)),
            ("Joint command (lift)", lambda: joint_pub.publish(msg_joint)),
            ("Navigate to Anchor A", lambda: anchor_pub.publish(msg_anchor))
        ]
        
        for name, test_func in tests:
            print(f"\n{name}...")
            test_func()
            time.sleep(0.5)
            print(f"  ✓ Sent")
        
        print("\n✓ All test commands sent successfully!")
        
        node.destroy_node()
        rclpy.shutdown()
        return True
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("ROS 2 Communication Test Suite")
    print("=" * 60)
    print("\nMake sure ROS 2 is sourced: source /opt/ros/jazzy/setup.bash")
    print("For full testing, run the simulation node first:")
    print("  python stretch_ros2_sim.py")
    print("=" * 60 + "\n")
    
    if not test_imports():
        sys.exit(1)
    
    if not test_topics():
        sys.exit(1)
    
    print("\n" + "=" * 60)
    if input("Send test commands? (y/n): ").strip().lower() == 'y':
        if not send_test_commands():
            sys.exit(1)
    
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print("✓ ROS 2 imports: OK")
    print("✓ Topic creation: OK")
    print("\nNext steps:")
    print("1. Start simulation: python stretch_ros2_sim.py")
    print("2. Run controller: python stretch_keyboard_controller.py")
    print("=" * 60 + "\n")


if __name__ == '__main__':
    main()
