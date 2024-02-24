cmake_minimum_required(VERSION 3.5)
project(mecanumbot_control)

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

# Disable boost/shared_ptr in pluginlib while building on rpi4
add_compile_definitions(PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mecanumbot_control_node
  src/mecanumbot_control_node.cpp
)
ament_target_dependencies(
  mecanumbot_control_node
  controller_manager
  realtime_tools
  rclcpp
)

install(TARGETS
  mecanumbot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
