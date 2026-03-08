"""Launch RViz visualization of the Otto Starter URDF model.

Usage:
    ros2 launch otto_description display.launch.py
    ros2 launch otto_description display.launch.py variant:=biped
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('otto_description')
    urdf_file = os.path.join(pkg, 'urdf', 'otto_starter.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'otto.rviz')

    variant_arg = DeclareLaunchArgument(
        'variant', default_value='wheeled',
        description='Robot variant: wheeled or biped')

    robot_description = ParameterValue(
        Command(['xacro "', urdf_file,
                 '" variant:=', LaunchConfiguration('variant')]),
        value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    # Sensor visualiser — publishes LED/sonar/line sensor markers for RViz
    visualizer = Node(
        package='otto_bringup',
        executable='otto_visualizer.py',
        name='otto_visualizer',
        output='screen')

    return LaunchDescription([
        variant_arg,
        robot_state_pub,
        joint_state_pub,
        visualizer,
        rviz,
    ])
