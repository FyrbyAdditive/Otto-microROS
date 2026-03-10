#!/bin/bash
# Entrypoint for Otto ROS2 Docker containers.
# Sources ROS2 + workspace, then runs the provided command.

set -e

source /opt/ros/jazzy/setup.bash
source /opt/otto/ros2_ws/install/setup.bash

exec "$@"
