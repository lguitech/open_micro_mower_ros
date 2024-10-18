cmake_minimum_required(VERSION 3.0.2)
project(mower_biz)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  mower_comm
  teb_local_planner
  mower_msgs
)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS  ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mower_msgs message_runtime
)

include_directories(
  include
  include/freeroute
  include/behavior
  include/control
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src/eigen-3.4.0)

add_executable(${PROJECT_NAME}_node 
  src/biz_node.cpp
  src/mr_state.cpp
  src/mr_biz.cpp
    
  src/freeroute/astar.cpp
  src/freeroute/mr_routedata.cpp

  src/behavior/mr_behav_base.cpp 
  src/behavior/mr_behav_dummy.cpp 
  src/behavior/mr_behav_dock.cpp
  src/behavior/mr_behav_fault.cpp
  src/behavior/mr_behav_gohome.cpp
  src/behavior/mr_behav_idle.cpp
  src/behavior/mr_behav_locate.cpp
  src/behavior/mr_behav_stuck.cpp
  src/behavior/mr_behav_transfer.cpp
  src/behavior/mr_behav_work.cpp
  src/behavior/mr_behav_struggle.cpp

  src/control/mr_controller_tool.cpp 
  src/control/mr_mower_controller.cpp
  src/control/mr_teb_controller.cpp
  src/control/mr_dock_controller.cpp
  
)

target_link_libraries(${PROJECT_NAME}_node
  teb_local_planner
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})