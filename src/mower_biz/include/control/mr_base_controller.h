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

#ifndef __MR_BASE_CONTROLLER_H__
#define __MR_BASE_CONTROLLER_H__

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#define ROUTE_TYPE_STRUGGLE 0
#define ROUTE_TYPE_GOHOME  	1
#define ROUTE_TYPE_TRANSFER 2
#define ROUTE_TYPE_DOCK 	3

class mr_base_controller {
protected:
    ros::NodeHandle handle;
    ros::NodeHandle nh_private;

	ros::Publisher pub_cmd_vel;
	
	double max_linear_speed;
	double max_angular_speed;
	double wheel_distance;

    bool isFinished = false;

public:

    mr_base_controller(const mr_base_controller&) = delete;
    mr_base_controller& operator=(const mr_base_controller&) = delete;

    virtual ~mr_base_controller() = default;

	mr_base_controller() : nh_private("~")
	{
		pub_cmd_vel = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);		
	}


	virtual void set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance) = 0;

	void stop_robot()
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0;
		cmd_vel.angular.z = 0;
		pub_cmd_vel.publish(cmd_vel);
	}

	void forward_midspeed()
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = max_linear_speed / 2;
		cmd_vel.linear.y = cmd_vel.linear.z = 0;
		cmd_vel.angular.z = 0;
		pub_cmd_vel.publish(cmd_vel);
	}		

	void pub_control(double linear_speed, double angular_speed)
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = linear_speed;
		cmd_vel.angular.z = angular_speed;
		pub_cmd_vel.publish(cmd_vel);
	}

};


#endif
