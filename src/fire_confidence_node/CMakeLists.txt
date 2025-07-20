cmake_minimum_required(VERSION 3.8)
project(fire_confidence_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  fire_confidence_node/fire_confidence_node.py
  DESTINATION lib/${PROJECT_NAME}
)

