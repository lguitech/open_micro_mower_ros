cmake_minimum_required(VERSION 3.0.2)
project(mower_comm)


find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mower_comm
  CATKIN_DEPENDS roscpp

)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  set(LIBPROJ  "/usr/lib/x86_64-linux-gnu/libproj.so")           
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")  
  set(LIBPROJ  "/usr/lib/aarch64-linux-gnu/libproj.so") 
endif()


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 include_directories(${OpenCV_INCLUDE_DIRS})
)

add_library(${PROJECT_NAME}
    src/mr_coord.cpp
    src/mr_util.cpp
    src/cjson.cpp
    src/cjsonobject.cpp
    src/minilzo.cpp
    src/mr_serialport.cpp
    src/mr_parsetool.cpp

)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${LIBPROJ}
  ${OpenCV_LIBRARIES}
)


install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)