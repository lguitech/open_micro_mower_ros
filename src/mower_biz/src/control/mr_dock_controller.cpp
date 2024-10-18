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


#include "mr_dock_controller.h"
#include "mr_navi_types.h"
#include "mr_util.h"


mr_dock_controller* mr_dock_controller::getInstance()
{
    static mr_dock_controller instance;
    return &instance;
}

void mr_dock_controller::cb_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
    config.reconfigure(reconfig);
}



mr_dock_controller::mr_dock_controller() : mr_base_controller()
{
    config.loadRosParamFromNodeHandle(nh_private);

	config.obstacles.min_obstacle_dist = 0.2;
	config.robot.max_vel_x = 0.1;
	config.robot.max_vel_x_backwards = 0.1;
	config.robot.max_vel_theta = 0.37;

    visual = TebVisualizationPtr(new TebVisualization(nh_private, config));
    robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(nh_private, config);

    if (config.hcp.enable_homotopy_class_planning) {
        planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
    }
    else {
        planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
    }
    
}


mr_dock_controller::~mr_dock_controller()
{

}

void mr_dock_controller::set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance)
{
	this->max_linear_speed = max_linear_speed;
	this->max_angular_speed = max_angular_speed;
	this->wheel_distance = wheel_distance;
}

void mr_dock_controller::set_pose(const geometry_msgs::Pose& msg) 
{
    double yaw = tf2::getYaw(msg.orientation);
    poseStart = {msg.position.x, msg.position.y, yaw};
}

void mr_dock_controller::set_path(nav_msgs::Path& path, int route_type)
{
	geometry_msgs::PoseStamped& finalPose = path.poses.back();

	double thetaGoal = tf::getYaw(finalPose.pose.orientation);
	poseGoal = {finalPose.pose.position.x, finalPose.pose.position.y, thetaGoal};				
}

void mr_dock_controller::reset_path()
{

}

bool mr_dock_controller::has_arrived()
{
    double dis = std::hypot(poseGoal.x() - poseStart.x(), poseGoal.y() - poseStart.y());
	double diffAngle = poseGoal.theta() - poseStart.theta();
	if (dis <= THRESHOLD_ARRIVED_DIST && diffAngle <= THRESHOLD_ARRIVED_ANGLE) {
		isFinished = true;
	}
	else {
		isFinished = false;
	}
	return isFinished;
}

int mr_dock_controller::compute_control_command()
{
	if (has_arrived()) {
		stop_robot();
		return 0;
	}


	bool result = planner->plan(poseStart, poseGoal);
	planner->visualize();

	double vx, vy, omega;
	bool ret = planner->getVelocityCommand(vx, vy, omega, config.trajectory.control_look_ahead_poses);    

	geometry_msgs::Twist cmd_vel;    
	cmd_vel.linear.x = vx;
	cmd_vel.angular.z = omega;  
	pub_cmd_vel.publish(cmd_vel);

	return 0;
}

bool mr_dock_controller::get_process_info()
{
    return isFinished;
}

