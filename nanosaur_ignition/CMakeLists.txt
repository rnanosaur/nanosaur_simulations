cmake_minimum_required(VERSION 3.5)

project(nanosaur_ignition)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY gui launch urdf
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()