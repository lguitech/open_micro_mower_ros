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


#include "mr_behav_work.h"

mr_behav_work::mr_behav_work()
{
	m_pController = mr_mower_controller::getInstance();
}
mr_behav_work::~mr_behav_work()
{
    
}

int mr_behav_work::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    m_counter++;
    if (m_counter >=10) {
        m_counter = 0;
        ROS_INFO_STREAM("perform Behavior Work......");
    }
	double detect_threshold, avoid_obstacle_distance;
	getParameter(detect_threshold, avoid_obstacle_distance);
	double detect_distance = detect_threshold + DIFF_BASE_LINK + RADIUS_BASE_LINK;
	
	if (getCurrGoalWaypoint() == -1) {
		return RESULT_FINISHED;
	}

	robot_poseCurr = poseCurr;

	if (needAvoidObstacle(detect_distance)) {
		return RESULT_OBSTACLE;
	}

    m_pController->set_pose(poseCurr);
    int curr_goal_waypoint = m_pController->compute_control_command();
	setCurrGoalWaypoint(curr_goal_waypoint);

    bool ret = m_pController->get_process_info();
	if (ret) {
		return RESULT_FINISHED;
	}
	else {
		return RESULT_DEFAULT;
	}
}

void mr_behav_work::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_down();
	cutter_rotate();

	double max_linear_speed, max_angular_speed, wheel_distance;
	getControlParameter(max_linear_speed, max_angular_speed, wheel_distance);
	m_pController->set_parameter(max_linear_speed, max_angular_speed, wheel_distance);
	int index = getCurrGoalWaypoint();
	m_pController->set_goal_waypoint(getCurrGoalWaypoint());
}

void mr_behav_work::onStop()
{
	putter_up();
	cutter_stop();
}

void mr_behav_work::onPause()
{
	m_pController->stop_robot();	
	putter_up();
	cutter_stop();
}

void mr_behav_work::onResume()
{
	putter_down();
	cutter_rotate();
}

void mr_behav_work::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{

}


void mr_behav_work::setWorkPath(nav_msgs::Path& path)
{
	m_pController->set_path(path);

	setTrajectory(path.poses);
	setCurrGoalWaypoint(0);
}

int mr_behav_work::calcNearestWayPoint(std::vector<geometry_msgs::PoseStamped>& vec_trajectory)
{
    double min_dis = DBL_MAX;
    int min_index = 0;
	
	int curr_goal_waypoint = getCurrGoalWaypoint();

    int start = std::max(0, curr_goal_waypoint - NEAREST_WAYPOINT_THRESHOLD);
    int end = std::min((int)vec_trajectory.size()-1, curr_goal_waypoint + NEAREST_WAYPOINT_THRESHOLD);
    for (int i=start; i<=end; i++) {
        geometry_msgs::Point& ptIter = vec_trajectory.at(i).pose.position;
        double dis = mower::PointDistance(
			robot_poseCurr.position.x, robot_poseCurr.position.y, ptIter.x, ptIter.y)    ;
        if (dis < min_dis) {
            min_dis = dis;
            min_index = i;
        }
    }
    return min_index;
}

void mr_behav_work::getNearWayPointList(std::vector<geometry_msgs::PoseStamped>& vec_trajectory,
										double detect_distance, std::vector<int> & vecWayPoint)
{
    if (vec_trajectory.size() == 0) {
        return;
    }
    
    int startIndex = calcNearestWayPoint(vec_trajectory);
    int endIndex = (int)(vec_trajectory.size());
	double sum_dis = 0;

	geometry_msgs::Pose posePrev = vec_trajectory.at(startIndex).pose;
	vecWayPoint.push_back(startIndex);

    for (int index = startIndex+1; index<endIndex; index++) {
        geometry_msgs::Pose poseCurr = vec_trajectory.at(index).pose;
		
		vecWayPoint.push_back(index);

		double dis = mower::PointDistance(posePrev.position.x, posePrev.position.y,
										  poseCurr.position.x, poseCurr.position.y);
		sum_dis += dis;

        if (sum_dis > detect_distance) {
            break;
        }
		posePrev = poseCurr;
    }
}

void mr_behav_work::getBaseCenter(mr_point& base_center)
{
    double yaw = tf::getYaw(robot_poseCurr.orientation);

    double dx = DIFF_BASE_LINK * cos(yaw);
    double dy = DIFF_BASE_LINK * sin(yaw);
    base_center.x = robot_poseCurr.position.x + dx;
    base_center.y = robot_poseCurr.position.y + dy;
}

bool mr_behav_work::needAvoidObstacle(double detect_distance)
{

	mr_point base_center;
	getBaseCenter(base_center);

    double min_dis = mr_route_data::getInstance()->minDistanceObstacle(base_center);

	double detect_threshold, avoid_obstacle_distance;
	getParameter(detect_threshold, avoid_obstacle_distance);


    if (min_dis > avoid_obstacle_distance) {
        return false;
    }

	std::vector<geometry_msgs::PoseStamped> vec_trajectory = getTrajectory();

	std::vector<int> vecNearWayPoint;
    getNearWayPointList(vec_trajectory, detect_distance, vecNearWayPoint);

	
    for (int index : vecNearWayPoint) {
		geometry_msgs::PoseStamped pose = vec_trajectory.at(index);
		mr_point pointOdom(pose.pose.position.x, pose.pose.position.y);
		if (mr_route_data::getInstance()->queryObstacle_odom(pointOdom)) {
			setCollisionIndex(index);
            return true;
        }
    }
    return false;
}


