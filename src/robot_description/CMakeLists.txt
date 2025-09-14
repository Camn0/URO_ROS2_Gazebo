cmake_minimum_required(VERSION 3.5)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
