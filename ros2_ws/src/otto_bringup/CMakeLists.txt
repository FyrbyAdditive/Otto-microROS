cmake_minimum_required(VERSION 3.8)
project(otto_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/otto_odom_publisher.py
  scripts/ultrasonic_to_laserscan.py
  scripts/otto_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
