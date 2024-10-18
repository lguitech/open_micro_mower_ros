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


#include "mr_behav_fault.h"

mr_behav_fault::mr_behav_fault()
{

}
mr_behav_fault::~mr_behav_fault()
{
    
}


int mr_behav_fault::performBehavior(const geometry_msgs::Pose& poseCurr)
{

}

void mr_behav_fault::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();
}

void mr_behav_fault::onStop()
{

}

void mr_behav_fault::onPause()
{

}

void mr_behav_fault::onResume()
{

}

void mr_behav_fault::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{

}
