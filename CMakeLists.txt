cmake_minimum_required(VERSION 3.22)
project(rc_channels_to_cmd)

# ──────────────────────────────────────────────────────────────────────────────
# Compiler flags
# ──────────────────────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wdouble-promotion -Wfloat-equal -Wshadow)

# ──────────────────────────────────────────────────────────────────────────────
# Dependencies
# ──────────────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(loc_modes_srvs REQUIRED)
find_package(emrs_locomotion_control REQUIRED)
find_package(emrs_suspension_control REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crsf_receiver_msg REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosidl_default_generators REQUIRED)
 

# ──────────────────────────────────────────────────────────────────────────────
# Component shared library
# ──────────────────────────────────────────────────────────────────────────────
add_library(RcChannelsToCmd_Node SHARED src/rc_channels_to_cmd.cpp)

target_link_libraries(RcChannelsToCmd_Node PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  realtime_tools::realtime_tools
  emrs_locomotion_control::emrs_locomotion_control_node
  emrs_suspension_control::emrs_suspension_control_node
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${crsf_receiver_msg_TARGETS}
  ${loc_modes_srvs_TARGETS}
)

target_include_directories(
  RcChannelsToCmd_Node
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)


add_library(RcChannelsToCmd SHARED src/component.cpp)
target_include_directories(
  RcChannelsToCmd
  PRIVATE
  include
)
target_link_libraries(
  RcChannelsToCmd
  RcChannelsToCmd_Node
  rclcpp_components::component
)

rclcpp_components_register_node(
  RcChannelsToCmd
  PLUGIN "rc_channels_to_cmd::RcChannelsToCmd"
  EXECUTABLE RcChannelsToCmdExecutable
)

# ──────────────────────────────────────────────────────────────────────────────
# Install
# ──────────────────────────────────────────────────────────────────────────────
install(TARGETS RcChannelsToCmd_Node RcChannelsToCmd
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

# ──────────────────────────────────────────────────────────────────────────────
# Export
# ──────────────────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  realtime_tools
  std_msgs
  sensor_msgs
  geometry_msgs
  crsf_receiver_msg
  loc_modes_srvs
  emrs_locomotion_control
  emrs_suspension_control
)

# ──────────────────────────────────────────────────────────────────────────────
# Testing
# ──────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()