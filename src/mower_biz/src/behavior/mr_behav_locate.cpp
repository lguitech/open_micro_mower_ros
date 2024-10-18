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



#include "mr_behav_locate.h"

mr_behav_locate::mr_behav_locate()
{
    heading_inited = false;
    loc_index = LOCATION_INDEX_INVALID;

	m_pController = mr_mower_controller::getInstance();
}

mr_behav_locate::~mr_behav_locate()
{
    
}


int mr_behav_locate::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    m_counter++;
    if (m_counter >=10) {
        m_counter = 0;
        ROS_INFO_STREAM("perform Behavior Locating......");
    }    

    if ((loc_index == LOCATION_INDEX_FIXED || loc_index == LOCATION_INDEX_FLOAT) && !heading_inited) 
    {
        m_pController->forward_midspeed();
    }
    return false;
}

void mr_behav_locate::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();

	double max_linear_speed, max_angular_speed, wheel_distance;
	getControlParameter(max_linear_speed, max_angular_speed, wheel_distance);
	m_pController->set_parameter(max_linear_speed, max_angular_speed, wheel_distance);
}

void mr_behav_locate::onStop()
{

}

void mr_behav_locate::onPause()
{
	m_pController->stop_robot();
}

void mr_behav_locate::onResume()
{

}

void mr_behav_locate::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{

}