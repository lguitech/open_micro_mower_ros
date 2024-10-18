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

#ifndef __MR_BEHAV_DOCK_H__
#define __MR_BEHAV_DOCK_H__

#include "mr_behav_base.h"

class mr_behav_dock : public mr_behav_base  {
private:
	mr_dock_controller* m_pController;

	geometry_msgs::Pose robot_poseCurr;
	geometry_msgs::Pose robot_poseGoal;

public:
    mr_behav_dock();
    ~mr_behav_dock();

	int performBehavior(const geometry_msgs::Pose& poseCurr) override;
	void onStart(const geometry_msgs::Pose* poseCurr = nullptr, 
				 const geometry_msgs::Pose* poseGoal = nullptr) override;
	void onStop() override;
	void onPause() override;
	void onResume() override;
	void onUpdateGoal(const geometry_msgs::Pose& poseGoal) override;

};

#endif