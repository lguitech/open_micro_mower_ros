cmake_minimum_required(VERSION 3.0.2)
project(mower_location)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  mower_comm
  sensor_msgs
  mower_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mower_msgs message_runtime
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src/eigen-3.4.0)

add_executable(${PROJECT_NAME}_node 
  src/location_node.cpp
  src/mr_location.cpp
  src/FusionMain.cpp
  src/kalman_filter1.cpp
  src/kalman_filter3.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} 
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})