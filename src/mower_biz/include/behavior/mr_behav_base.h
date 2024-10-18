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

#ifndef __MR_BEHAV_BASE_H__
#define __MR_BEHAV_BASE_H__
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include "mr_navi_types.h"
#include "mr_routedata.h"
#include "mr_mower_controller.h"
#include "mr_teb_controller.h"
#include "mr_dock_controller.h"
#include "mr_routedata.h"

#include "mower_msgs/MowerChassisControl.h"


#define RESULT_DEFAULT 0
#define RESULT_FINISHED 1
#define RESULT_OBSTACLE 2

#define DIFF_BASE_LINK 0.2f
#define RADIUS_BASE_LINK 0.25f

class mr_behav_global {
public:
    static mr_behav_global* getInstance() {
        static mr_behav_global instance;
        return &instance;
    }

    ros::Publisher pub_chassis_control;	
	ros::Publisher pub_free_path;	

    double param_detect_threshold;
    double param_avoid_obstacle_distance;

	double param_max_linear_speed;
	double param_max_angular_speed;
    double param_wheel_distance;
	
	int collision_index;

	std::vector<geometry_msgs::PoseStamped> vec_trajectory;
	int curr_goal_waypoint = 0;

private:
    mr_behav_global() {} 
};


class mr_behav_base {
protected:	
	ros::Publisher& getChassisControlPublisher()
	{
		return mr_behav_global::getInstance()->pub_chassis_control;
	}

	ros::Publisher& getFreePathPublisher()
	{
		return mr_behav_global::getInstance()->pub_free_path;
	}

public:
    int m_counter;
	
	void setChassisControlPublisher(ros::Publisher& pub) 
	{
		mr_behav_global::getInstance()->pub_chassis_control = pub;
	}	

	void setFreePathPublisher(ros::Publisher& pub) 
	{
		mr_behav_global::getInstance()->pub_free_path = pub;
	}	

	void setParameter(double detect_threshold, double avoid_obstacle_distance)
	{
		mr_behav_global::getInstance()->param_detect_threshold = detect_threshold;
		mr_behav_global::getInstance()->param_avoid_obstacle_distance = avoid_obstacle_distance;
	}

	void getParameter(double& detect_threshold, double& avoid_obstacle_distance)
	{
		detect_threshold = mr_behav_global::getInstance()->param_detect_threshold;
		avoid_obstacle_distance = mr_behav_global::getInstance()->param_avoid_obstacle_distance;
	}

	void setControlParameter(double max_linear_speed, double max_angular_speed, double wheel_distance)
	{
		mr_behav_global::getInstance()->param_max_linear_speed = max_linear_speed;
		mr_behav_global::getInstance()->param_max_angular_speed = max_angular_speed;
		mr_behav_global::getInstance()->param_wheel_distance = wheel_distance;
	}

	void getControlParameter(double& max_linear_speed, double& max_angular_speed, double& wheel_distance)
	{
		max_linear_speed = mr_behav_global::getInstance()->param_max_linear_speed;
		max_angular_speed = mr_behav_global::getInstance()->param_max_angular_speed;
		wheel_distance = mr_behav_global::getInstance()->param_wheel_distance;
	}

	void setCollisionIndex(int index) 
	{
		mr_behav_global::getInstance()->collision_index = index;
	}

	int getCollisionIndex()
	{
		return mr_behav_global::getInstance()->collision_index;
	}


	void setCurrGoalWaypoint(int index) 
	{
		mr_behav_global::getInstance()->curr_goal_waypoint = index;
	}

	int getCurrGoalWaypoint()
	{
		return mr_behav_global::getInstance()->curr_goal_waypoint;
	}

	void setTrajectory(std::vector<geometry_msgs::PoseStamped>& vec_trajectory)
	{
		mr_behav_global::getInstance()->vec_trajectory = vec_trajectory;
	}

	std::vector<geometry_msgs::PoseStamped>& getTrajectory()
	{
		return mr_behav_global::getInstance()->vec_trajectory;
	}

	void putter_up();
	void putter_down();
	void cutter_rotate();
	void cutter_stop();

    virtual ~mr_behav_base() {}

	virtual int performBehavior(const geometry_msgs::Pose& poseCurr) = 0;
	virtual void onStart(const geometry_msgs::Pose* poseCurr = nullptr, 
						 const geometry_msgs::Pose* poseGoal = nullptr) = 0;
	virtual void onStop() = 0;
	virtual void onPause() = 0;
	virtual void onResume() = 0;
	virtual void onUpdateGoal(const geometry_msgs::Pose& poseGoal) = 0;

	virtual void setWorkPath(nav_msgs::Path& path) {}
};

#endif