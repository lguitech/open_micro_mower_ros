#!/bin/bash

# Source ROS setup files
source /opt/ros/melodic/setup.bash
source ~/open_micro_mower_ros/devel/setup.bash

rm -rf ~/.ros/log/*

# Start your ROS node or launch file
roslaunch ~/open_micro_mower_ros/launch/run.launch
