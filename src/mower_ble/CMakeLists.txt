
cmake_minimum_required(VERSION 3.0.2)
project(mower_ble)

set(CMAKE_CXX_STANDARD 11)

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBZIP REQUIRED libzip)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mower_comm
  mower_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mower_msgs message_runtime
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CURL_LIBRARY /usr/lib/aarch64-linux-gnu/libcurl.so)
    set(LIBZIP_LIBRARY /usr/lib/aarch64-linux-gnu/libzip.so)
    include_directories(/usr/include)
else()
    find_package(CURL REQUIRED)
    set(CURL_LIBRARY ${CURL_LIBRARIES})
    set(LIBZIP_LIBRARY ${LIBZIP_LIBRARIES})
    include_directories(${CURL_INCLUDE_DIRS} ${LIBZIP_INCLUDE_DIRS})
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node 
  src/ble_node.cpp
  src/mr_ble.cpp
  src/mr_bletool.cpp
  src/mr_ota.cpp
)


target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${CURL_LIBRARY}
  ${LIBZIP_LIBRARY}
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})