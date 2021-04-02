cmake_minimum_required(VERSION 3.5)
project(mecanumbot_teleop)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mecanumbot_teleop
  src/node_main.cpp
  src/mecanumbot_teleop.cpp
  src/mecanumbot_joystick.cpp
)
target_include_directories(
  mecanumbot_teleop
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  mecanumbot_teleop
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  mecanumbot_teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
