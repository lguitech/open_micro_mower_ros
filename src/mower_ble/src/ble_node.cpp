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
#include "mr_ble.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mower_ble");
    ROS_INFO("ble node started");

    ros::NodeHandle handle;
    ros::NodeHandle nh_private("~");

    double max_linear_speed = 0;
    double wheel_distance = 0;    

    handle.getParam("/mower/max_linear_speed", max_linear_speed);
    handle.getParam("/mower/wheel_distance", wheel_distance);

    assert(max_linear_speed != 0);
    assert(wheel_distance != 0);

    mr_class_ble::getInstance()->init(max_linear_speed, wheel_distance);

    ros::spin(); 

    mr_class_ble::getInstance()->unInit();

    return 0;
}