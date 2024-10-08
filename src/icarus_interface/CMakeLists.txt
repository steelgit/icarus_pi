cmake_minimum_required(VERSION 3.5)
project(icarus_interface)

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
find_package(pid_messages REQUIRED)



add_library(icarus_interface SHARED
src/encoder_control.cpp
src/Icarus_HW_Interface.cpp 
src/motor_control.cpp 
src/rotary_encoder.cpp
src/wheel.cpp)

target_link_libraries(icarus_interface pigpiod_if2)

target_include_directories(
  icarus_interface
  PRIVATE
  include
)
ament_target_dependencies(
  icarus_interface
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
  control_toolbox
  pid_messages
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)


install(
  TARGETS icarus_interface
  DESTINATION lib
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  icarus_interface
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)


ament_package()
