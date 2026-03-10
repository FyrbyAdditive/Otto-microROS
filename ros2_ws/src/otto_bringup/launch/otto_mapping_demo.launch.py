"""Mapping demo: micro-ROS bringup + slam_toolbox + RViz.

Usage:
    ros2 launch otto_bringup otto_mapping_demo.launch.py
"""

import os
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('otto_bringup')
    desc_pkg = get_package_share_directory('otto_description')

    # Include the main bringup launch (agent:=none — agent is managed separately)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'otto_microros.launch.py')),
        launch_arguments={'agent': 'none'}.items())

    # slam_toolbox in online async mode (lifecycle node)
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[os.path.join(bringup_pkg, 'config', 'slam_toolbox.yaml')],
        output='screen')

    # Auto-activate: configure on startup, then activate once configured
    configure_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == slam,
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    emit_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == slam,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    # RViz with mapping display
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'otto.rviz')],
        output='screen')

    return LaunchDescription([
        bringup,
        slam,
        configure_slam,
        emit_configure,
        rviz,
    ])
