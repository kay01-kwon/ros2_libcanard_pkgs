cmake_minimum_required(VERSION 3.8)
project(ros2_libcanard_test)

set(CMAKE_CXX_STANDARD 17)

set(esc_test_src
  include/utils/rc_converter.cpp
  include/utils/single_mode.cpp
  include/node/esc_test.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros2_libcanard_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_library(
  esc_test_lib
  ${esc_test_src}
)

ament_target_dependencies(
  esc_test_lib
  rclcpp
  ros2_libcanard_msgs
  mavros_msgs
)
add_executable(
  esc_test_node
  src/esc_test_node.cpp
)

target_link_libraries(
  esc_test_node
  esc_test_lib
)

install(
  TARGETS esc_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
