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


#include "mr_behav_idle.h"

mr_behav_idle::mr_behav_idle()
{
    count = 0;
    last_timestamp = 0;
	m_pController = mr_mower_controller::getInstance();
}
mr_behav_idle::~mr_behav_idle()
{
    
}


int mr_behav_idle::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    uint64_t time_in_ms = ros::Time::now().toNSec() / 1000000;

    if (time_in_ms - last_timestamp > 1000) {
        m_pController->stop_robot();
    }
    last_timestamp = time_in_ms;

	return false;
}

void mr_behav_idle::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();
}

void mr_behav_idle::onStop()
{

}

void mr_behav_idle::onPause()
{

}

void mr_behav_idle::onResume()
{

}

void mr_behav_idle::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{

}
