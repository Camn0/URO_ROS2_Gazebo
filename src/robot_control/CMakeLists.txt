cmake_minimum_required(VERSION 3.5)
project(robot_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Kita install setiap script satu per satu agar sintaksnya pasti benar

install(PROGRAMS
  src/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detection
)

install(PROGRAMS
  src/movement_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME movement_control
)

install(PROGRAMS
  src/pid_x.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_x
)

install(PROGRAMS
  src/pid_y.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_y
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
