cmake_minimum_required(VERSION 3.5)
project(robot_description)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and launch files
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

