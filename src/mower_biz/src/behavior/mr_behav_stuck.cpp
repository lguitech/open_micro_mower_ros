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


#include "mr_behav_stuck.h"

mr_behav_stuck::mr_behav_stuck()
{

}
mr_behav_stuck::~mr_behav_stuck()
{
    
}


int mr_behav_stuck::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    m_counter++;
    if (m_counter >=10) {
        m_counter = 0;
        ROS_INFO_STREAM("perform Behavior Stuck......");
    }
	return false;
}

void mr_behav_stuck::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();
}

void mr_behav_stuck::onStop()
{

}

void mr_behav_stuck::onPause()
{

}

void mr_behav_stuck::onResume()
{

}

void mr_behav_stuck::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{

}