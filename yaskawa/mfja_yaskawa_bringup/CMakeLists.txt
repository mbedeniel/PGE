cmake_minimum_required(VERSION 3.8)
project(mfja_yaskawa_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/move_joint.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
