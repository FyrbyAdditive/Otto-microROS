"""Main bringup launch: micro-ROS agent + URDF + odometry + scan converter.

Usage:
    ros2 launch otto_bringup otto_microros.launch.py
    ros2 launch otto_bringup otto_microros.launch.py agent_port:=9999
    ros2 launch otto_bringup otto_microros.launch.py agent:=none   # agent already running
    ros2 launch otto_bringup otto_microros.launch.py agent:=docker # use Docker agent
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_pkg = get_package_share_directory('otto_description')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'otto_starter.urdf.xacro')

    agent_port_arg = DeclareLaunchArgument(
        'agent_port', default_value='8888',
        description='UDP port for micro-ROS agent')

    variant_arg = DeclareLaunchArgument(
        'variant', default_value='wheeled',
        description='Robot variant: wheeled or biped')

    agent_arg = DeclareLaunchArgument(
        'agent', default_value='native',
        description='Agent mode: native, docker, or none')

    robot_description = ParameterValue(
        Command([
            'xacro "', urdf_file,
            '" variant:=', LaunchConfiguration('variant')]),
        value_type=str)

    # micro-ROS agent via Docker (default)
    agent_docker = ExecuteProcess(
        cmd=[
            'docker', 'run', '--rm', '--net=host',
            'microros/micro-ros-agent:jazzy',
            'udp4', '--port', LaunchConfiguration('agent_port'), '-v4'],
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('agent'), 'docker')))

    # micro-ROS agent — installed via apt (ros-jazzy-micro-ros-agent)
    agent_native = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'udp4', '--port', LaunchConfiguration('agent_port'), '-v4'],
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('agent'), 'native')))

    # Robot state publisher (URDF -> TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    # Dead-reckoning odometry
    odom_pub = Node(
        package='otto_bringup',
        executable='otto_odom_publisher.py',
        name='otto_odom_publisher',
        output='screen')

    # Ultrasonic Range -> LaserScan
    scan_converter = Node(
        package='otto_bringup',
        executable='ultrasonic_to_laserscan.py',
        name='ultrasonic_to_laserscan',
        parameters=[{'mode': 'passthrough'}],
        output='screen')

    # LED ring + sonar visualizer (publishes MarkerArray for RViz)
    visualizer = Node(
        package='otto_bringup',
        executable='otto_visualizer.py',
        name='otto_visualizer',
        output='screen')

    return LaunchDescription([
        agent_port_arg,
        variant_arg,
        agent_arg,
        agent_docker,
        agent_native,
        robot_state_pub,
        odom_pub,
        scan_converter,
        visualizer,
    ])
