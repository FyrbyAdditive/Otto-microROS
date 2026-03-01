"""Mapping demo: micro-ROS bringup + slam_toolbox + RViz.

Usage:
    ros2 launch otto_bringup otto_mapping_demo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('otto_bringup')
    desc_pkg = get_package_share_directory('otto_description')

    # Include the main bringup launch
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'otto_microros.launch.py')))

    # slam_toolbox in online async mode
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(bringup_pkg, 'config', 'slam_toolbox.yaml')],
        output='screen')

    # RViz with mapping display
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'otto.rviz')],
        output='screen')

    return LaunchDescription([
        bringup,
        slam,
        rviz,
    ])
