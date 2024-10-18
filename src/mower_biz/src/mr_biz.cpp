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
#include "mr_biz.h"
#include "mr_teb_controller.h"


void mr_class_biz::cb_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    MR_State::getInstance()->setMapData(*msg);

}
void mr_class_biz::cb_obstacle(const mower_msgs::MowerObstacle::ConstPtr& msg)
{
    MR_State::getInstance()->setObstacleData(*msg);
}


void mr_class_biz::cb_home_update(const geometry_msgs::Pose::ConstPtr& msg)
{
	MR_State::getInstance()->update_home(*msg);
}


void mr_class_biz::cb_map_meta(const mower_msgs::MowerMapMeta::ConstPtr& msg)
{
    MR_State::getInstance()->set_home(msg->home);
}

void mr_class_biz::optimize_path_display(const nav_msgs::Path& input, nav_msgs::Path& output)
{
    
    int size = input.poses.size();
    if (size ==0) {
        return;
    }
    output.header = input.header;
    const geometry_msgs::PoseStamped& posePrev = input.poses.at(0);
    output.poses.emplace_back(posePrev);
    double yawPrev = tf::getYaw(posePrev.pose.orientation);

    for (int i=1; i<size-1; i++) {
        const geometry_msgs::PoseStamped& poseCurr = input.poses.at(i);
        double yawCurr = tf::getYaw(poseCurr.pose.orientation);
        double diff = fabs(yawCurr - yawPrev) * 180 / M_PI;
        if (diff > 2) {
            output.poses.emplace_back(poseCurr);
            yawPrev = yawCurr;
        }
    }

    output.poses.emplace_back(input.poses.at(size-1));
}

void mr_class_biz::cb_path(const mower_msgs::MowerPath::ConstPtr& msg)
{
    MR_State::getInstance()->set_work_path(*msg);
    for (const auto& path : msg->paths) {
        nav_msgs::Path path_optimized;
        optimize_path_display(path, path_optimized);
        pub_region_path.publish(path_optimized);
    }
}



void mr_class_biz::cb_gnss(const mower_msgs::MowerGnss::ConstPtr& msg)
{
    m_gnssMsg = *msg;

}

void mr_class_biz::cb_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_odomMsg = *msg;

    MR_State::getInstance()->set_location(m_gnssMsg.loc_index, m_gnssMsg.heading_inited, m_odomMsg);
}


void mr_class_biz::cb_work_control(const std_msgs::Int32::ConstPtr& msg)
{
    switch (msg->data) {
    case ROBOT_WORK_CONTROL_RESET:
        MR_State::getInstance()->on_cmd_reset();
        break;
    case ROBOT_WORK_CONTROL_GOHOME:
        MR_State::getInstance()->on_cmd_gohome();
        break;
    case ROBOT_WORK_CONTROL_START:
        MR_State::getInstance()->on_cmd_start();
        break;
    case ROBOT_WORK_CONTROL_PAUSE:
        MR_State::getInstance()->on_cmd_pause();
        break;
    default:
        break;
    }
}

void mr_class_biz::currentGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    MR_State::getInstance()->on_cmd_calc_route(m_odomMsg, *msg);
}


void mr_class_biz::init(bool is_simu, double max_linear_speed, double max_angular_speed,
    double wheel_distance, double detect_threshold, double avoid_obstacle_distance)
{
    MR_State::getInstance()->set_parameter(detect_threshold, avoid_obstacle_distance);
	MR_State::getInstance()->set_control_parameter(max_linear_speed, max_angular_speed, wheel_distance);
    MR_State::getInstance()->on_cmd_launch();

    std::string mower_gnss_topic;
    std::string mower_map_topic;
    std::string mower_map_meta_topic;
    std::string mower_obstacle_topic;

    std::string mower_work_control_topic;
    std::string mower_odom_topic;
    std::string mower_free_path_topic;
    std::string mower_work_path_topic;
    std::string mower_region_path_topic;
    std::string mower_chassis_control_topic;
    //std::string mower_cluster_obstacle_topic;
	std::string mower_home_update_topic;

    
    nh_private.param<std::string>("mower_gnss_topic",               mower_gnss_topic,             "/mower/gnss");
    nh_private.param<std::string>("mower_map_topic",                mower_map_topic,              "/mower/map");
    nh_private.param<std::string>("mower_map_meta_topic",           mower_map_meta_topic,         "/mower/map_meta");
    nh_private.param<std::string>("mower_obstacle_topic",           mower_obstacle_topic,         "/mower/obstacle");

    nh_private.param<std::string>("mower_work_control_topic",       mower_work_control_topic,     "/mower/work_control");

    nh_private.param<std::string>("mower_odom_topic",               mower_odom_topic,             "/mower/odom");
    nh_private.param<std::string>("mower_free_path_topic",          mower_free_path_topic,        "/mower/free_path");
    nh_private.param<std::string>("mower_work_path_topic",          mower_work_path_topic,        "/mower/work_path");
    nh_private.param<std::string>("mower_region_path_topic",        mower_region_path_topic,      "/mower/region_path");

    nh_private.param<std::string>("mower_chassis_control_topic",    mower_chassis_control_topic,  "/mower/chassis_control");

	nh_private.param<std::string>("mower_home_update_topic",   		mower_home_update_topic, 	  "/mower/home_update");
	

    sub_map = handle.subscribe<nav_msgs::OccupancyGrid>(mower_map_topic, 10, &mr_class_biz::cb_map, this);

    sub_mower_map_meta = handle.subscribe<mower_msgs::MowerMapMeta>(mower_map_meta_topic, 10, &mr_class_biz::cb_map_meta, this);           
    
    sub_path = handle.subscribe<mower_msgs::MowerPath>(mower_work_path_topic, 10, &mr_class_biz::cb_path, this);
    
    sub_work_control = handle.subscribe<std_msgs::Int32>(mower_work_control_topic, 10, &mr_class_biz::cb_work_control, this);
    
    sub_gnss = handle.subscribe<mower_msgs::MowerGnss>(mower_gnss_topic, 10, &mr_class_biz::cb_gnss, this);
    
    sub_odom = handle.subscribe<nav_msgs::Odometry>(mower_odom_topic, 10, &mr_class_biz::cb_odom, this);        
    
    sub_obstacle = handle.subscribe<mower_msgs::MowerObstacle>(mower_obstacle_topic, 10, &mr_class_biz::cb_obstacle, this);
    

    sub_current_goal = handle.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &mr_class_biz::currentGoalCallback, this);

	sub_home_update = handle.subscribe<geometry_msgs::Pose>(mower_home_update_topic, 10, &mr_class_biz::cb_home_update, this);

	

    pub_free_path = handle.advertise<nav_msgs::Path>(mower_free_path_topic, 10, true);

    MR_State::getInstance()->setFreePathPublisher(pub_free_path);

    pub_region_path = handle.advertise<nav_msgs::Path>(mower_region_path_topic, 10, true);

    pub_chassis_control = handle.advertise<mower_msgs::MowerChassisControl>(mower_chassis_control_topic, 10, true);
	
    pub_cmd_vel = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    
    MR_State::getInstance()->setChassisControlPublisher(pub_chassis_control);

}

void mr_class_biz::unInit()
{

}

