cmake_minimum_required(VERSION 3.8)
project(joystick_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# Include directory
include_directories(include)

# Executable target
add_executable(joystick_controller_node
  src/main.cpp
  src/joystick_controller.cpp
)

# Link dependencies
ament_target_dependencies(
  joystick_controller_node
  rclcpp
  geometry_msgs
  spdlog
)

target_link_libraries(joystick_controller_node
  fmt::fmt
)

target_include_directories(joystick_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Install target
install(
  TARGETS joystick_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
