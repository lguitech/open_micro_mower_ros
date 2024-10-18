#include <ros/ros.h>
#include "costmap_converter/mr_costmap_converter.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mower costmap converter node!");
    ros::NodeHandle handle;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO_STREAM("mower costmap converter started!");
    
    mr_costmap_converter converter;
    converter.init();

    ros::spin();

    converter.unInit();
    return 0;
}
