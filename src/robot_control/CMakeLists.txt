cmake_minimum_required(VERSION 3.5)
project(robot_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

install(
  PROGRAMS
    src/object_detection.py
    src/pid_controller.py
    src/movement_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

