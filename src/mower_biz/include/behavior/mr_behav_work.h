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

#ifndef __MR_BEHAV_WORK_H__
#define __MR_BEHAV_WORK_H__

#include "mr_behav_base.h"
class mr_behav_work : public mr_behav_base  {
private:
	int const NEAREST_WAYPOINT_THRESHOLD = 5;
	geometry_msgs::Pose robot_poseCurr;
	mr_mower_controller* m_pController = nullptr;

	int calcNearestWayPoint(std::vector<geometry_msgs::PoseStamped>& vec_trajectory);
	void getNearWayPointList(std::vector<geometry_msgs::PoseStamped>& vec_trajectory,
							 double detect_distance, std::vector<int> & vecWayPoint);
	bool needAvoidObstacle(double detect_distance);
	void getBaseCenter(mr_point& base_center);
public:
    mr_behav_work();
    ~mr_behav_work();

	int performBehavior(const geometry_msgs::Pose& poseCurr) override;
	void onStart(const geometry_msgs::Pose* poseCurr = nullptr, 
				 const geometry_msgs::Pose* poseGoal = nullptr) override;

	void onStop() override;
	void onPause() override;
	void onResume() override;
	void onUpdateGoal(const geometry_msgs::Pose& poseGoal) override;

	void setWorkPath(nav_msgs::Path& path);

};

#endif



