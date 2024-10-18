cmake_minimum_required(VERSION 3.0.2)
project(mower_map)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  mower_comm
  mower_msgs
  pcl_ros
  pcl_conversions  
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mower_msgs message_runtime
)

include_directories(
  include
  include/gridmap
  include/route
  ${catkin_INCLUDE_DIRS}
  include_directories(${OpenCV_INCLUDE_DIRS})
)

add_executable(${PROJECT_NAME}_node 
  src/map_node.cpp
  src/mr_map.cpp
  src/mr_mappub.cpp
  src/mr_cluster.cpp
  src/mr_obstacle_convert.cpp
  
  src/gridmap/mr_mapproc.cpp
  src/gridmap/mr_preproc_tool.cpp
  src/gridmap/mr_genmap_tool.cpp
  
  
  src/route/mr_route.cpp
    
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})