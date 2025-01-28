cmake_minimum_required(VERSION 3.5)
project(ws2812b_controller)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ws2812b_node src/ws2812b_node.cpp)
ament_target_dependencies(ws2812b_node rclcpp std_msgs)

install(TARGETS ws2812b_node DESTINATION lib/${PROJECT_NAME})

ament_package()
