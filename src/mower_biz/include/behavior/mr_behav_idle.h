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

#ifndef __MR_BEHAV_IDLE_H__
#define __MR_BEHAV_IDLE_H__

#include "mr_behav_base.h"
class mr_behav_idle : public mr_behav_base  {
protected:
	mr_mower_controller* m_pController;
    uint64 last_timestamp = 0;
    int count;
public:
    mr_behav_idle();
    ~mr_behav_idle();

	int performBehavior(const geometry_msgs::Pose& poseCurr) override;
	void onStart(const geometry_msgs::Pose* poseCurr = nullptr, 
				 const geometry_msgs::Pose* poseGoal = nullptr) override;

	void onStop() override;
	void onPause() override;
	void onResume() override;
	void onUpdateGoal(const geometry_msgs::Pose& poseGoal) override;

};

#endif