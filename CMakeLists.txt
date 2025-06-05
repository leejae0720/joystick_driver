cmake_minimum_required(VERSION 3.8)
project(joystick_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_executable(joystick_controller_node src/joystick_controller.cpp)

ament_target_dependencies(
  joystick_controller_node
  rclcpp
  geometry_msgs
  spdlog
)

target_link_libraries(joystick_controller_node fmt::fmt)

install(
  TARGETS joystick_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
