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

#ifndef __MR_BIZ_H__
#define  __MR_BIZ_H__

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include "mr_coord.h"
#include "mr_state.h"

#include "mower_msgs/MowerPath.h"
#include "mower_msgs/MowerObstacle.h"
#include "mower_msgs/MowerGnss.h"
#include "mower_msgs/GetFreeRoute.h"
#include "mower_msgs/GetFreeRouteRequest.h"
#include "mower_msgs/GetFreeRouteResponse.h"
#include "mower_msgs/MowerChassisControl.h"
#include "mower_msgs/MowerMapMeta.h"



class mr_class_biz {
private:
    bool is_simu = false;
    ros::NodeHandle handle;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_map;
    ros::Subscriber sub_mower_map_meta;
    ros::Subscriber sub_path;
    ros::Subscriber sub_work_control;
    ros::Subscriber sub_gnss;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_marker;
    ros::Subscriber sub_current_goal;


	ros::Subscriber sub_home_update;

    ros::Publisher pub_work_state;
    ros::Publisher pub_free_path;
    ros::Publisher pub_region_path;
    ros::Publisher pub_chassis_control;
    ros::Publisher pub_cmd_vel;

    nav_msgs::Odometry m_odomMsg;
    mower_msgs::MowerGnss m_gnssMsg;


    void cb_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void cb_obstacle(const mower_msgs::MowerObstacle::ConstPtr& msg);
    void cb_map_meta(const mower_msgs::MowerMapMeta::ConstPtr& msg);

    void optimize_path_display(const nav_msgs::Path& input, nav_msgs::Path& output);
    void cb_path(const mower_msgs::MowerPath::ConstPtr& msg);
    void cb_gnss(const mower_msgs::MowerGnss::ConstPtr& msg);
    void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void cb_work_control(const std_msgs::Int32::ConstPtr& msg);
    void currentGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void cb_home_update(const geometry_msgs::Pose::ConstPtr& msg);

    mr_class_biz() : nh_private("~")  {}
public:
    static mr_class_biz* getInstance() {
        static mr_class_biz instance;
        return &instance;
    }

    mr_class_biz(const mr_class_biz&) = delete;
    mr_class_biz& operator=(const mr_class_biz&) = delete;

    ~mr_class_biz() {};

    void init(bool is_simu, double max_linear_speed, double max_angular_speed,
             double wheel_distance, double detect_threshold, 
             double avoid_obstacle_distance);

    void unInit();

};


#endif

