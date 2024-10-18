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


#ifndef __MR_STATE_H__
#define __MR_STATE_H__

#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "mower_msgs/MowerPath.h"
#include "mower_msgs/MowerGnss.h"
#include "mower_msgs/MowerObstacle.h"
#include "mr_navi_types.h"
#include "mr_behav_base.h"
#include "mr_behav_dummy.h"
#include "mr_behav_dock.h"
#include "mr_behav_fault.h"
#include "mr_behav_idle.h"
#include "mr_behav_locate.h"
#include "mr_behav_stuck.h"
#include "mr_behav_transfer.h"
#include "mr_behav_work.h"
#include "mr_behav_gohome.h"
#include "mr_behav_struggle.h"
#include "mr_routedata.h"


class MR_State 
{
private:
    ros::NodeHandle nh_private;
    MR_State();
    
    std::thread threadMain;
    void threadFunction();
    bool threadRunning;
    std::recursive_mutex mutex_state;

	const double NEAR_HOME_THRESHOLD = 1.0;
    const double ARRIVED_THRESHOLD = 0.5;
    int loc_index;
    bool heading_inited;


    geometry_msgs::Pose poseHome;
    
    int curr_state;
	mr_behav_dummy behav_dummy;
    mr_behav_dock behav_dock;
    mr_behav_fault behav_fault;
    mr_behav_idle behav_idle;
    mr_behav_locate behav_locate;
    mr_behav_stuck behav_stuck;
    mr_behav_transfer behav_transfer;
    mr_behav_work behav_work;
    mr_behav_gohome behav_gohome;
    mr_behav_struggle behav_struggle;


    int state_before_pause;
    int state_before_stuck;
    int state_before_locate;

    int num_region;
    int curr_region;

    std::vector<nav_msgs::Path> m_vecPath;

    bool map_initialized;


    void pause_behav();
    void resume_behav();
    void switch_behav(int new_state);
    bool has_gnss_fixed();
    bool has_arrived_goal(geometry_msgs::Pose& poseCurr, geometry_msgs::Pose& poseGoal);
    bool near_home(geometry_msgs::Pose& poseCurr,geometry_msgs::Pose& poseGoal);
    void on_event_lost_fix();
    void on_event_location_fixed();
    void on_behav_finished();
	void on_behav_obstacle();
    bool is_fixed();

    geometry_msgs::Pose robot_poseCurr;

public:
    static MR_State* getInstance();
    mr_behav_base* mr_behav;

    ~MR_State();

    void set_parameter(double detect_threshold, double avoid_obstacle_distance); 
	void set_control_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance);
    void setChassisControlPublisher(ros::Publisher& pub);
	void setFreePathPublisher(ros::Publisher& pub);
    int get_robot_state();
    
    void set_location(int loc_index, bool heading_inited, nav_msgs::Odometry& msg_odom);
    void setMapData(const nav_msgs::OccupancyGrid& msg);
    void setObstacleData(const mower_msgs::MowerObstacle& msg);
    void set_work_path(const mower_msgs::MowerPath& msg);
    void set_home(const geometry_msgs::Point& home);
	void update_home(const geometry_msgs::Pose& poseHome);

    void on_cmd_calc_route(const nav_msgs::Odometry& odom_msg, const geometry_msgs::PoseStamped& goal);
    void on_cmd_launch();
    void on_cmd_reset();
    void on_cmd_start();
    void on_cmd_pause();
    void on_cmd_gohome();
};


#endif