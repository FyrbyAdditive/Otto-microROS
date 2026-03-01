"""Keyboard teleop for Otto Starter.

Usage:
    ros2 launch otto_bringup otto_teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'speed': 0.1,       # m/s — conservative for small robot
                'turn': 0.5,        # rad/s
            }]),
    ])
