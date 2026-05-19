cmake_minimum_required(VERSION 3.8)
project(crsf_joy_translator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies -----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Custom packages (adjust package names to match your workspace)
find_package(crsf_receiver_msg REQUIRED)          # CRSFChannels16
find_package(emrs_locomotion_control REQUIRED)
find_package(emrs_suspension_control REQUIRED)
find_package(emrs_joy_translator REQUIRED)        # reuses Button class
find_package(loc_modes_srvs REQUIRED)

# ---- Library (composable node) ----------------------------------------------
add_library(${PROJECT_NAME}_component SHARED
  src/crsf_joy_translator.cpp
  src/crsf_component.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  realtime_tools
  geometry_msgs
  std_msgs
  sensor_msgs
  crsf_receiver_msg
  emrs_locomotion_control
  emrs_suspension_control
  emrs_joy_translator
  loc_modes_srvs
)

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "joystick::CRSFJoyTranslator"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ---- Install ----------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
