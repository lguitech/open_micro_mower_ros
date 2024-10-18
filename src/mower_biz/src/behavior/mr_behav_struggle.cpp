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


#include "mr_behav_struggle.h"


mr_behav_struggle::mr_behav_struggle() : nh_private("~") 
{
	m_pController = mr_teb_controller::getInstance();
}
mr_behav_struggle::~mr_behav_struggle()
{
    
}


int mr_behav_struggle::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    m_counter++;
    if (m_counter >=10) {
        m_counter = 0;
        ROS_INFO_STREAM("perform Behavior Struggle......");
    }
	
	robot_poseCurr = poseCurr;

    m_pController->set_pose(poseCurr);
    m_pController->compute_control_command();
    bool ret = m_pController->get_process_info();
	if (ret) {
		return RESULT_FINISHED;
	}
	else {
		return RESULT_DEFAULT;
	}

}

void mr_behav_struggle::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();

	isPaused = false;
	robot_poseCurr = *poseCurr;
	startRouteCycle();
}

void mr_behav_struggle::onStop()
{
	isPaused = true;
	stopRouteCycle();
}

void mr_behav_struggle::onPause()
{
	m_pController->stop_robot();
	isPaused = true;
}

void mr_behav_struggle::onResume()
{
	isPaused = false;
}

void mr_behav_struggle::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{
	//robot_poseGoal = poseGoal;
}

void mr_behav_struggle::cb_mainRouteCycle(const ros::TimerEvent&)
{
	if (isPaused) {
		return;
	}
	nav_msgs::Path freePath;

	while(true) {
		bool succ = mr_route_data::getInstance()->doRouteCalc(robot_poseCurr, robot_poseGoal, freePath);
		if (succ) {
			m_pController->set_path(freePath, ROUTE_TYPE_STRUGGLE);
			getFreePathPublisher().publish(freePath);
			break;
		}
		else {
			//ros::Duration(1.0).sleep(); 
			int curr_goal_waypoint = find_new_goal(getCurrGoalWaypoint(), robot_poseGoal);
			setCurrGoalWaypoint(curr_goal_waypoint);
			if (curr_goal_waypoint == -1) {
				m_pController->skip_to_final();
				return;
			}
		}
	}
}


void mr_behav_struggle::startRouteCycle()
{
	int curr_goal_waypoint = find_new_goal(getCollisionIndex(), robot_poseGoal);
	setCurrGoalWaypoint(curr_goal_waypoint);
	if (curr_goal_waypoint == -1) {
		m_pController->skip_to_final();
		return;
	}

	ros::TimerEvent event;
    cb_mainRouteCycle(event);

	cycle_timer = nh_private.createTimer(ros::Duration(RECAL_DURATION),
				 &mr_behav_struggle::cb_mainRouteCycle, this, false, false); //oneshot = false, start = false
    cycle_timer.start();
}

void mr_behav_struggle::stopRouteCycle()
{
    if (cycle_timer.hasStarted()) {
        cycle_timer.stop();
    }
}


int mr_behav_struggle::find_new_goal(int curr_value, geometry_msgs::Pose& poseGoal)
{
	if (curr_value == -1) {
		return -1;
	}
	std::vector<geometry_msgs::PoseStamped> vec_trajectory = getTrajectory();

	int end_index = (int)vec_trajectory.size() - 1;
	int free_counter = 0;
	
	for (int index = curr_value; index<=end_index; index++) {
        const geometry_msgs::Pose& poseIter = vec_trajectory.at(index).pose;
        mr_point pointTestObstacle(poseIter.position.x, poseIter.position.y);
        if (!mr_route_data::getInstance()->queryObstacle_odom(pointTestObstacle)) {
			free_counter ++;
		}
		else {
			free_counter = 0;
		}
		if (free_counter >= CONTINUE_FREE_COUNTER) {
			poseGoal = poseIter;
			return index;
		}
	}
	return -1;
}



