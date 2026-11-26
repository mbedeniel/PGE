cmake_minimum_required(VERSION 3.16)
project(mfja_yaskawa_gz_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED src/yaskawa_gazebo_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Only the header of gz_ros2_control::GazeboSimSystemInterface is needed: use the include directory
# of the exported gz_ros2_control target without linking its libraries, which Gazebo loads itself.
get_target_property(gz_ros2_control_include_dirs
  gz_ros2_control::gz_ros2_control-system INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} PRIVATE ${gz_ros2_control_include_dirs})
target_link_libraries(${PROJECT_NAME} PUBLIC
  gz-sim::gz-sim
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Makes the class visible to the gz_ros2_control plugin loader.
pluginlib_export_plugin_description_file(gz_ros2_control mfja_yaskawa_gz_hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_command test/test_velocity_command.cpp)
  target_include_directories(test_velocity_command PRIVATE include)
endif()

ament_package()
