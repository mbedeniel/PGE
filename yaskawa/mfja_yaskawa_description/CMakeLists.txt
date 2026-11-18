cmake_minimum_required(VERSION 3.8)
project(mfja_yaskawa_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo launch meshes robots ros2_control rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Lets Gazebo resolve package://mfja_yaskawa_description/... mesh URIs
# once the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
