cmake_minimum_required(VERSION 3.0.2)
project(mower_chassis)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  mower_comm
  sensor_msgs
  mower_msgs
  pcl_ros
  pcl_conversions  
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mower_msgs message_runtime
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/chassis_node.cpp
  src/mr_chassis.cpp
  src/mr_sonarfilter.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} 
   ${PCL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})