/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROS] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#include <cmath>
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>


#include "mower_msgs/MowerChassisState.h"
#include "mower_msgs/MowerSonar.h"

#include "mr_navi_types.h"
#include "mr_chassis.h"


ros::Publisher pub_chassisStatus;


bool is_global_simu = false;

double wheel_distance       = 0.4;
double wheel_radius         = 0.1;

void publish_chassis_status(mr_chassis_status& chassis_status)
{
    mower_msgs::MowerChassisState msg_chassis;

    msg_chassis.auto_mode = chassis_status.auto_mode;
    msg_chassis.battery_charge_state = chassis_status.battery_charge_state;
    msg_chassis.battery_volt = chassis_status.battery_volt;
    msg_chassis.cutter_state = chassis_status.cutter_state;
    msg_chassis.fault_code = chassis_status.fault_code;   
    msg_chassis.left_speed = chassis_status.left_speed;
    msg_chassis.putter_state = chassis_status.putter_state;
    msg_chassis.right_speed = chassis_status.right_speed;
    msg_chassis.safe_edge_back = chassis_status.safe_edge_back;
    msg_chassis.safe_edge_front = chassis_status.safe_edge_front;
    msg_chassis.safe_edge_left = chassis_status.safe_edge_left;
    msg_chassis.safe_edge_right = chassis_status.safe_edge_right;
    msg_chassis.soc = chassis_status.soc;
    msg_chassis.brake_state = chassis_status.brake_state;

    msg_chassis.sonar_front_left    = chassis_status.sonar_front_left;
    msg_chassis.sonar_front_center  = chassis_status.sonar_front_center;
    msg_chassis.sonar_front_right   = chassis_status.sonar_front_right;
    msg_chassis.sonar_rear_left     = chassis_status.sonar_rear_left;
    msg_chassis.sonar_rear_center   = chassis_status.sonar_rear_center;
    msg_chassis.sonar_rear_right    = chassis_status.sonar_rear_right;

    pub_chassisStatus.publish(msg_chassis);    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mower_chassis");
    ROS_INFO("chassis node started");
    
    std::string mower_chassis_state_topic;
    std::string mower_sonar_marker_topic;    

    std::string mower_cluster_obstacle_topic;
    std::string mower_cluster_marker_topic;
    
    ros::NodeHandle handle;
    ros::NodeHandle nh_private("~");

    handle.getParam("/mower/global_simu", is_global_simu);
    handle.getParam("/mower/wheel_distance", wheel_distance);
    handle.getParam("/mower/wheel_radius", wheel_radius);


    mr_class_chassis::getInstance()->init(is_global_simu, wheel_distance, wheel_radius);

    nh_private.getParam("mower_chassis_state_topic", mower_chassis_state_topic);
    nh_private.getParam("mower_sonar_marker_topic", mower_sonar_marker_topic);

    nh_private.getParam("mower_cluster_marker_topic", mower_cluster_marker_topic);
    nh_private.getParam("mower_cluster_obstacle_topic", mower_cluster_obstacle_topic);

    pub_chassisStatus = handle.advertise<mower_msgs::MowerChassisState>(mower_chassis_state_topic, 10);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(5.0).sleep(); 

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        mr_chassis_status chassis_status;
        mr_class_chassis::getInstance()->get_chassis_status(&chassis_status);

        publish_chassis_status(chassis_status);

        loop_rate.sleep();
    }

    spinner.stop();

    mr_class_chassis::getInstance()->unInit();

    return 0;
}