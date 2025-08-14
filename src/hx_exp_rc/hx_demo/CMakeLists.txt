cmake_minimum_required(VERSION 3.0.2)
project(hx_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  hx_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    hx_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Basic position control demo
add_executable(basic_position_control src/basic_position_control.cpp)
target_link_libraries(basic_position_control ${catkin_LIBRARIES})
add_dependencies(basic_position_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Circle trajectory demo  
add_executable(circle_trajectory src/circle_trajectory.cpp)
target_link_libraries(circle_trajectory ${catkin_LIBRARIES})
add_dependencies(circle_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Controller comparison demo
add_executable(controller_comparison src/controller_comparison.cpp)
target_link_libraries(controller_comparison ${catkin_LIBRARIES})
add_dependencies(controller_comparison ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Ground effect experiment
add_executable(ground_effect_experiment src/ground_effect_experiment.cpp)
target_link_libraries(ground_effect_experiment ${catkin_LIBRARIES})
add_dependencies(ground_effect_experiment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install
install(TARGETS basic_position_control circle_trajectory controller_comparison ground_effect_experiment
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)