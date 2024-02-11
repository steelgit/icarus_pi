cmake_minimum_required(VERSION 3.5)
project(diff_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_toolbox REQUIRED)



add_library(diff_interface SHARED src/diffdrive_MyBot.cpp src/wheel.cpp src/motor_control.cpp src/rotary_encoder.cpp)

target_link_libraries(diff_interface pigpiod_if2)

target_include_directories(
  diff_interface
  PRIVATE
  include
)
ament_target_dependencies(
  diff_interface
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
  control_toolbox
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)


install(
  TARGETS diff_interface
  DESTINATION lib
)

install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  diff_interface
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)


ament_package()
