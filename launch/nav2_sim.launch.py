"""Launch Nav2 for the Stretch 3 MuJoCo simulation.

Spawns the Nav2 navigation stack (controller, planner, smoother, behaviors,
BT navigator, lifecycle manager) plus a static ``map -> odom`` identity
transform. There is no AMCL or map_server: the simulator publishes ground-truth
odometry, and the identity map transform anchors it in the ``map`` frame.

Run after the simulator node is up (``make sim``):

    ros2 launch launch/nav2_sim.launch.py

Nav2 outputs ``cmd_vel`` remapped to ``/stretch/cmd_vel``, which the simulator
already subscribes to.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/stretch/cmd_vel')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/stretch/cmd_vel')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server', 'smoother_server', 'planner_server',
                'behavior_server', 'bt_navigator',
            ],
        }],
    )

    return LaunchDescription([
        map_to_odom,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
