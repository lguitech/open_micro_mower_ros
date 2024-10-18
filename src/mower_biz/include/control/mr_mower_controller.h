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

#ifndef __MR_MOWER_CONTROLLER_H__
#define __MR_MOWER_CONTROLLER_H__

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "mr_navi_types.h"
#include "mr_controller_tool.h"
#include "mr_base_controller.h"

class mr_mower_controller : public mr_base_controller
{
private:

    std::ofstream log_file;
    double const DETECT_OBSTACLE_LENGTH = 2.0;

    int const LOOK_FORWARD_STEPS = 5;
    double const LOOK_FORWARD_DISTANCE = 1.0;
    double const THRESHOLD_ARRIVED = 0.2;

    mr_pose last_pose;

    std::vector<geometry_msgs::PoseStamped> vec_trajectory;

    int goal_waypoint;
    int look_waypoint;
    bool isResetCalcGoal;

    MR_ControllerTool controllerTool;
private:
    double toNormalRegion(double angle);

    double normalizeDiff(double diff);

    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
    void getMRPose(const geometry_msgs::PoseStamped& input, mr_pose& output);

    void compute_control_command_normal(mr_cmd& cmd);
    void cal_look_forward_position(int& tIndex, double& tx, double& ty);

    bool has_arrived_goal();
    std::string getFormatTime();

    void convert_cmd_vel(double left_speed, double right_speed, 
						 double& linear_speed, double& angular_speed);

	void reset();
	mr_mower_controller();
public:
    ~mr_mower_controller();
    mr_mower_controller(const mr_mower_controller&) = delete;
    mr_mower_controller& operator=(const mr_mower_controller&) = delete;

    static mr_mower_controller* getInstance();

	void set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance) override;
	
	void skip_to_final();
	void set_goal_waypoint(int index);
    
	void set_path(nav_msgs::Path& path);
	void reset_path();
	void set_pose(const geometry_msgs::Pose& pose);
    int compute_control_command();
	bool get_process_info();


};



#endif