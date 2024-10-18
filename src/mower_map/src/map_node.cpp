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

#include <csignal>
#include <stdexcept>
#include <ros/ros.h>
#include "mr_map.h"


std::string filespecRawMap;

void doHandle()
{
    try {
        std::remove(filespecRawMap.c_str());
    }
    catch (const std::exception& e) {}
}

void signalHandler(int signal) {
    ROS_ERROR("Caught signal: %d", signal);
    switch (signal) {
        case SIGSEGV:
            doHandle();
            ROS_ERROR("Caught SIGSEGV: Segmentation Fault");
            break;
        case SIGABRT:
            doHandle();
            ROS_ERROR("Caught SIGABRT: Abort");
            break;
        case SIGFPE:
            doHandle();
            ROS_ERROR("Caught SIGFPE: Floating Point Exception");
            break;
        case SIGINT:
            ROS_INFO("Caught SIGINT: Interrupt from Ctrl+C");
            ros::shutdown();
            break;
        default:
            ROS_ERROR("Caught unknown signal: %d", signal);
            break;
    }
    exit(signal);    
}

void errHandle()
{   
    doHandle();
    exit(-1);
}


int main(int argc, char** argv)
{
    bool is_simu = false;
    double detect_threshold = 1.2;

    ros::init(argc, argv, "mower_map");
    ROS_INFO("map node started");
     
    ros::NodeHandle handle;
    ros::NodeHandle nh_private("~");

    std::signal(SIGSEGV, signalHandler);
    std::signal(SIGABRT, signalHandler);
    std::signal(SIGFPE, signalHandler);

    handle.getParam("/mower/global_simu", is_simu);
    handle.getParam("/mower/sonar_distance", detect_threshold);

    nh_private.getParam("filespec", filespecRawMap);

    mr_class_map::getInstance()->init(is_simu, detect_threshold, filespecRawMap, errHandle);

    ros::spin();

    mr_class_map::getInstance()->unInit();

    return 0;
}