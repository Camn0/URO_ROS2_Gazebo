cmake_minimum_required(VERSION 3.5)
project(robot_navigation)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)

# Install Python nodes
install(
  DIRECTORY src
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

