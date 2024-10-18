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


#include "mr_behav_transfer.h"

mr_behav_transfer::mr_behav_transfer() : nh_private("~") 
{
	m_pController = mr_teb_controller::getInstance();
}
mr_behav_transfer::~mr_behav_transfer()
{
    
}


int mr_behav_transfer::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    m_counter++;
    if (m_counter >=10) {
        m_counter = 0;
        ROS_INFO_STREAM("perform Behavior Transfer......");
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

void mr_behav_transfer::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();

	isPaused = false;
	robot_poseCurr = *poseCurr;
	robot_poseGoal = *poseGoal;
	startRouteCycle();
}

void mr_behav_transfer::onStop()
{
	isPaused = true;
	stopRouteCycle();
}

void mr_behav_transfer::onPause()
{
	m_pController->stop_robot();
	isPaused = true;
}

void mr_behav_transfer::onResume()
{
	isPaused = false;
}

void mr_behav_transfer::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{
	robot_poseGoal = poseGoal;
}

void mr_behav_transfer::cb_mainRouteCycle(const ros::TimerEvent&)
{
	if (isPaused) {
		return;
	}
	nav_msgs::Path freePath;
	bool result = mr_route_data::getInstance()->doRouteCalc(robot_poseCurr, robot_poseGoal, freePath);

    if (result) {
        m_pController->set_path(freePath, ROUTE_TYPE_TRANSFER);
		getFreePathPublisher().publish(freePath);
    }
    else {
        m_pController->reset_path();
	}	
}


void mr_behav_transfer::startRouteCycle()
{
	ros::TimerEvent event;
    cb_mainRouteCycle(event);

	cycle_timer = nh_private.createTimer(ros::Duration(RECAL_DURATION),
				 &mr_behav_transfer::cb_mainRouteCycle, this, false, false); 
    cycle_timer.start();
}

void mr_behav_transfer::stopRouteCycle()
{
    if (cycle_timer.hasStarted()) {
        cycle_timer.stop();
    }
}