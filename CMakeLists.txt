cmake_minimum_required(VERSION 3.5)
project(ws2812b)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add your source files here
add_executable(ws2812b_node src/led.cpp) # Replace 'main.cpp' with your actual file name
ament_target_dependencies(ws2812b_node rclcpp)

install(TARGETS
  ws2812b_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
