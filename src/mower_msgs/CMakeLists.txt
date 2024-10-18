cmake_minimum_required(VERSION 3.0.2)
project(mower_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  MowerGnss.msg
  MowerSonar.msg
  MowerChassisState.msg
  MowerChassisControl.msg
  MowerMapUpdate.msg
  MowerObstacle.msg
  MowerPath.msg
  MowerMapMeta.msg
)

add_service_files(
  FILES
  GetFreeRoute.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
