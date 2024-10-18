cmake_minimum_required(VERSION 3.0.2)
project(mower_aruco)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	geometry_msgs
	visualization_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	cv_bridge
	image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
	src/aruco_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
