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
#include "mr_location.h"

int main(int argc, char** argv)
{
    bool is_simu = false;
    ros::init(argc, argv, "mower_gnss");
    ROS_INFO("gnss node started");
    
    ros::NodeHandle handle;
    ros::NodeHandle nh_private("~");
    handle.getParam("/mower/global_simu", is_simu);
    
    mr_class_location::getInstance()->init(is_simu);

    ros::spin();
    
    mr_class_location::getInstance()->unInit();

    return 0;
}
