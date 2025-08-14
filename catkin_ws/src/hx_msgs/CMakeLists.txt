cmake_minimum_required(VERSION 3.0.2)
project(hx_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

# Message files
add_message_files(
  FILES
  UAVCommand.msg
  UAVState.msg
  UAVControlState.msg
  AirSimState.msg
  ControllerOutput.msg
  TrajectoryPoint.msg
  DetectionInfo.msg
)

# Service files
add_service_files(
  FILES
  UAVService.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)