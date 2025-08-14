cmake_minimum_required(VERSION 3.0.2)
project(hx_uav_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  hx_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

# Dynamic reconfigure (config file not created yet)
# generate_dynamic_reconfigure_options(
#   cfg/hx_controller.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hx_uav_control
  CATKIN_DEPENDS 
    roscpp 
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    hx_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Libraries (temporarily exclude AirSim interface due to header issues)
add_library(hx_uav_control
  src/hx_uav_controller.cpp
  # src/hx_airsim_interface.cpp  # Commented out due to AirSim header issues
)

add_dependencies(hx_uav_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hx_uav_control
  ${catkin_LIBRARIES}
)

# Main control node
add_executable(hx_uav_control_node src/hx_uav_control_node.cpp)
add_dependencies(hx_uav_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hx_uav_control_node
  hx_uav_control
  ${catkin_LIBRARIES}
)

# AirSim interface node (temporarily disabled)
# add_executable(hx_airsim_interface_node src/hx_airsim_interface_node.cpp)
# add_dependencies(hx_airsim_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(hx_airsim_interface_node
#   hx_uav_control
#   ${catkin_LIBRARIES}
# )

# Utilities
add_executable(hx_command_pub src/utils/hx_command_pub.cpp)
add_dependencies(hx_command_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hx_command_pub ${catkin_LIBRARIES})

# Install
install(TARGETS hx_uav_control hx_uav_control_node hx_command_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)