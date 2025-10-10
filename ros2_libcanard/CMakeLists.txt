cmake_minimum_required(VERSION 3.8)
project(ros2_libcanard)

set(CMAKE_CXX_STANDARD 17)

# CANARD sources
set(CANARD_INCLUDE 
include/canard_impl)

set(CANARD_SRC
  ${CANARD_INCLUDE}/canard_internals/canard.c
  ${CANARD_INCLUDE}/canard_interface/canard_interface.cpp
  ${CANARD_INCLUDE}/driver/socketcan.c
)
# DSDL generated sources
set(DSDL_GENERATED_SRC
    include/dsdl_generated/uavcan.protocol.NodeStatus.c
    include/dsdl_generated/uavcan.protocol.GetNodeInfo_res.c
    include/dsdl_generated/uavcan.protocol.GetNodeInfo_req.c
    include/dsdl_generated/uavcan.equipment.esc.RawCommand.c
    include/dsdl_generated/uavcan.equipment.esc.RPMCommand.c
    include/dsdl_generated/uavcan.equipment.esc.Status.c
    include/dsdl_generated/uavcan.protocol.dynamic_node_id.Allocation.c
    include/dsdl_generated/uavcan.protocol.param.GetSet_req.c
    include/dsdl_generated/uavcan.protocol.param.GetSet_res.c
    include/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_req.c
    include/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_res.c
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ros2_libcanard_msgs REQUIRED)

# Compiler flags
add_definitions(-DDRONECAN_CXX_WRAPPERS)

include_directories(
  include
  ${CANARD_INCLUDE}
  ${CANARD_INCLUDE}/canard_internals
  ${CANARD_INCLUDE}/driver
  ${CANARD_INCLUDE}/canard_interface
  ${CANARD_INCLUDE}/canard
  include/dsdl_generated
  ${MSG_LIB_INCLUDE}
)

add_library(canard_lib
  ${CANARD_SRC}
)

add_library(dsdl_generated_lib
  ${DSDL_GENERATED_SRC}
)

add_executable(ros2_libcanard_node
  src/ros2_libcanard_node.cpp
  include/ros2_libcanard/ros2_libcanard.cpp
)

ament_target_dependencies(ros2_libcanard_node 
rclcpp 
std_msgs
rcl_interfaces
ros2_libcanard_msgs
)

target_link_libraries(ros2_libcanard_node
  canard_lib
  dsdl_generated_lib
)

install(TARGETS
  ros2_libcanard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
