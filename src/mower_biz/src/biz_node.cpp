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


#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "mr_biz.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mower_biz");
    ROS_INFO("biz node started");

    std::string mower_work_state_topic;

    ros::NodeHandle handle;
    ros::NodeHandle nh_private("~");

    bool is_simu = false;
    double max_linear_speed;
    double max_angular_speed;
    double wheel_distance = 0.4;
    double detect_threshold = 1.2;
    double avoid_obstacle_distance = 0.6;

    handle.getParam("/mower/global_simu", is_simu);
    handle.getParam("/mower/max_linear_speed", max_linear_speed);
    handle.getParam("/mower/max_angular_speed", max_angular_speed);
    
    handle.getParam("/mower/wheel_distance", wheel_distance);
    handle.getParam("/mower/sonar_distance", detect_threshold);
    handle.getParam("/mower/avoid_obstacle_distance", avoid_obstacle_distance);

    nh_private.getParam("mower_work_state_topic", mower_work_state_topic);
    ros::Publisher pub_work_state = handle.advertise<std_msgs::Int32>(mower_work_state_topic, 10, true);

    mr_class_biz::getInstance()->init(is_simu, max_linear_speed, max_angular_speed,
        wheel_distance, detect_threshold, avoid_obstacle_distance);
    ros::AsyncSpinner spinner(3);
    spinner.start();



    ros::Rate loop_rate(1);
    while(ros::ok()) {
        int state = MR_State::getInstance()->get_robot_state();
        std_msgs::Int32 msg;
        msg.data = state;
        pub_work_state.publish(msg);

        loop_rate.sleep();
    }
    spinner.stop();

    return 0;
}