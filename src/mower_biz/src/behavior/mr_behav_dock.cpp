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


#include "mr_behav_dock.h"

mr_behav_dock::mr_behav_dock()
{
	m_pController = mr_dock_controller::getInstance();
}

mr_behav_dock::~mr_behav_dock()
{
    
}

int mr_behav_dock::performBehavior(const geometry_msgs::Pose& poseCurr)
{
    m_counter++;
    if (m_counter >=10) {
        m_counter = 0;
        ROS_INFO_STREAM("perform Behavior Docking......");
    }    

	nav_msgs::Path freePath;

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = poseCurr;
	freePath.poses.emplace_back(pose_stamped);
	pose_stamped.pose = robot_poseGoal;
	freePath.poses.emplace_back(pose_stamped);

	m_pController->set_path(freePath, ROUTE_TYPE_DOCK);

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

void mr_behav_dock::onStart(const geometry_msgs::Pose* poseCurr, const geometry_msgs::Pose* poseGoal)
{
	putter_up();
	cutter_stop();
	robot_poseGoal = *poseGoal;
}

void mr_behav_dock::onStop()
{

}

void mr_behav_dock::onPause()
{
	m_pController->stop_robot();
}

void mr_behav_dock::onResume()
{

}

void mr_behav_dock::onUpdateGoal(const geometry_msgs::Pose& poseGoal)
{
	robot_poseGoal = poseGoal;
}


