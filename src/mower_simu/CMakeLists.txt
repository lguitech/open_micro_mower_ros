cmake_minimum_required(VERSION 3.0.2)
project(mower_simu)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
