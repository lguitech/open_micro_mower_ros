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

#include <ros/ros.h>
#include "mower_msgs/MowerChassisControl.h"
#include "mr_state.h"
#include "mr_util.h"
#include "std_msgs/Int32.h"

MR_State::MR_State(): nh_private("~")
{
    curr_state = ROBOT_STATE_IDLE;

    map_initialized = false;
    mr_behav = &behav_dummy;
    threadRunning = false;

    loc_index = LOCATION_INDEX_INVALID;
    heading_inited = false;
}

MR_State::~MR_State()
{
    void* pRet = NULL;
    threadRunning = false;
    if (threadMain.joinable()) {
        threadMain.join();
    }
}

void MR_State::set_parameter(double detect_threshold, double avoid_obstacle_distance)
{
	mr_behav->setParameter(detect_threshold, avoid_obstacle_distance);

}                

void MR_State::set_control_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance)
{
	mr_behav->setControlParameter(max_linear_speed, max_angular_speed, wheel_distance);
} 

MR_State* MR_State::getInstance()
{
    static MR_State instance;
    return &instance;
}


void MR_State::threadFunction()
{
    while(threadRunning) {
        mutex_state.lock();
		
        if (mr_behav != nullptr && curr_state != ROBOT_STATE_PAUSED) {
			if (mr_behav_locate* ptr = dynamic_cast<mr_behav_locate*>(mr_behav)) {
				ptr->heading_inited = this->heading_inited;
				ptr->loc_index = this->loc_index;
			}
			int result = mr_behav->performBehavior(robot_poseCurr);
			if (result == RESULT_FINISHED) {
				on_behav_finished();
			}
			else if (result == RESULT_OBSTACLE) {
				switch_behav(ROBOT_STATE_STRUGGLING);
			}
        }

        mutex_state.unlock();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MR_State::on_behav_obstacle()
{

}
void MR_State::on_behav_finished()
{
    switch(curr_state) {
    case ROBOT_STATE_DOCKING:
        ROS_INFO_STREAM("on_behav_finished docking");
        switch_behav(ROBOT_STATE_IDLE);
        break;
    case ROBOT_STATE_GOINGHOME:
        ROS_INFO_STREAM("on_behav_finished goinghome");
        switch_behav(ROBOT_STATE_DOCKING);
        break;
    case ROBOT_STATE_TRANSFERRING:
        ROS_INFO_STREAM("on_behav_finished transferring");
        switch_behav(ROBOT_STATE_WORKING);
        break;
    case ROBOT_STATE_WORKING:
        ROS_INFO_STREAM("on_behav_finished working currregion = " << curr_region);
        if (curr_region == num_region - 1) {
            curr_region = 0;
            switch_behav(ROBOT_STATE_GOINGHOME);
        }
        else {
            curr_region += 1;
            switch_behav(ROBOT_STATE_TRANSFERRING);
        }    
        break;    
    case ROBOT_STATE_STRUGGLING:
        ROS_INFO_STREAM("on_behav_finished struggling");
        switch_behav(ROBOT_STATE_WORKING);
        break;

    }
}

bool MR_State::has_gnss_fixed()
{
    if (loc_index == LOCATION_INDEX_FIXED || loc_index == LOCATION_INDEX_FLOAT)
    {
        return true;
    }
    else {
        return false;
    }
}



void MR_State::on_event_lost_fix()
{
    if (curr_state == ROBOT_STATE_GOINGHOME ||
        curr_state == ROBOT_STATE_TRANSFERRING ||
        curr_state == ROBOT_STATE_WORKING ||
        curr_state == ROBOT_STATE_STRUGGLING) 
    {
        state_before_locate = curr_state;
        switch_behav(ROBOT_STATE_LOCATING);
    }
}   


void MR_State::on_event_location_fixed()
{
    if (curr_state == ROBOT_STATE_LOCATING) {
        switch_behav(state_before_locate);
    }
}

bool MR_State::is_fixed()
{
    if ((loc_index == LOCATION_INDEX_FIXED || loc_index == LOCATION_INDEX_FLOAT) && heading_inited ) {
        return true;
    }   
    else {
        return false;
    }
}

void MR_State::set_location(int loc_index,bool heading_inited, nav_msgs::Odometry& msg_odom)
{
   std::lock_guard<std::recursive_mutex> lock(mutex_state);

    this->loc_index = loc_index;
    this->heading_inited = heading_inited;

    robot_poseCurr = msg_odom.pose.pose;

	
    if (is_fixed()) {
        on_event_location_fixed();
    }
    else {
        on_event_lost_fix();
    }   
}

bool MR_State::near_home(geometry_msgs::Pose& poseCurr,geometry_msgs::Pose& poseGoal)
{
    double dis = mower::PointDistance(poseCurr.position.x, poseCurr.position.y, poseGoal.position.x, poseGoal.position.y);    
    if (dis < NEAR_HOME_THRESHOLD) {
        return true;
    }
    else {
        return false;
    }
}


bool MR_State::has_arrived_goal(geometry_msgs::Pose& poseCurr,geometry_msgs::Pose& poseGoal)
{
    double dis = mower::PointDistance(poseCurr.position.x, poseCurr.position.y, poseGoal.position.x, poseGoal.position.y);    
    if (dis < ARRIVED_THRESHOLD) {
        return true;
    }
    else {
        return false;
    }
}


void MR_State::pause_behav()
{
    state_before_pause = curr_state;
    curr_state = ROBOT_STATE_PAUSED;
	mr_behav->onPause();
}

void MR_State::resume_behav()
{
    curr_state = state_before_pause;
	mr_behav->onResume();
}

void MR_State::switch_behav(int new_state)
{
    switch(new_state){
    case ROBOT_STATE_DOCKING:
		mr_behav->onStop();
        mr_behav = &behav_dock;
		mr_behav->onStart(nullptr, &poseHome);
        curr_state = new_state;
        break;    
    case ROBOT_STATE_FAULT:
		mr_behav->onStop();
        mr_behav = &behav_fault;
		mr_behav->onStart();
        curr_state = new_state;       
        break;    
    case ROBOT_STATE_IDLE:
		mr_behav->onStop();
        mr_behav = &behav_idle;
		mr_behav->onStart();
        curr_state = new_state;       
        break;    
    case ROBOT_STATE_LOCATING:
		mr_behav->onStop();
        mr_behav = &behav_locate;
		mr_behav->onStart();
        curr_state = new_state;       
        break;    
    case ROBOT_STATE_STUCK:
		mr_behav->onStop();
        mr_behav = &behav_stuck;
		mr_behav->onStart();
        curr_state = new_state;       
        break;    
    case ROBOT_STATE_STRUGGLING:
		mr_behav->onStop();
        mr_behav = &behav_struggle;
		mr_behav->onStart(&robot_poseCurr, nullptr);
        curr_state = new_state;      
		break;		
    case ROBOT_STATE_TRANSFERRING:
    {
        nav_msgs::Path& work_path = m_vecPath[curr_region];
        geometry_msgs::Pose robot_poseGoal = work_path.poses[0].pose;

        if (has_arrived_goal(robot_poseCurr, robot_poseGoal)) {
            ROS_INFO_STREAM("arrive to path start, swith to working state ...");
            switch_behav(ROBOT_STATE_WORKING);
        }
        else {
			mr_behav->onStop();
			mr_behav = &behav_transfer;
			mr_behav->onStart(&robot_poseCurr, &robot_poseGoal);
            curr_state = new_state;
        }
        break;    
    }
    case ROBOT_STATE_GOINGHOME:

        if (near_home(robot_poseCurr, poseHome)) {
            switch_behav(ROBOT_STATE_DOCKING);
        }
        else {
			mr_behav->onStop();
			mr_behav = &behav_gohome;
			mr_behav->onStart(&robot_poseCurr, &poseHome);
			curr_state = new_state;   
        }
        break;
 	
    case ROBOT_STATE_WORKING:

		mr_behav->onStop();
		mr_behav = &behav_work;
		
        if (curr_state == ROBOT_STATE_TRANSFERRING) {
            mr_behav->setWorkPath(m_vecPath.at(curr_region));
        }

		mr_behav->onStart();
		curr_state = new_state;

        break;    
    }
}

int MR_State::get_robot_state()
{
    return curr_state;
}


void MR_State::setFreePathPublisher(ros::Publisher& pub)
{
	mr_behav->setFreePathPublisher(pub);
}

void MR_State::setChassisControlPublisher(ros::Publisher& pub)
{
	mr_behav->setChassisControlPublisher(pub);
}


void MR_State::setMapData(const nav_msgs::OccupancyGrid& msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);
    switch_behav(ROBOT_STATE_IDLE);

    mr_route_data::getInstance()->setMapData(msg);

    map_initialized = true;
}

void MR_State::set_work_path(const mower_msgs::MowerPath& msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);
    switch_behav(ROBOT_STATE_IDLE);
    m_vecPath.clear();
    for (auto& path : msg.paths) {
        m_vecPath.emplace_back(path);
    }
    num_region = m_vecPath.size();
    curr_region = 0;
}

void MR_State::set_home(const geometry_msgs::Point& home)
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
	poseHome.position.x = home.x;
	poseHome.position.y = home.y;
	poseHome.position.z = home.z;

	double yawHome = 0; 
	tf2::Quaternion q;
	q.setRPY(0, 0, yawHome);

	poseHome.orientation.x = q.x();
	poseHome.orientation.y = q.y();
	poseHome.orientation.z = q.z();
	poseHome.orientation.w = q.w();
}

void MR_State::update_home(const geometry_msgs::Pose& poseHome)
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
	this->poseHome = poseHome;
	if (curr_state == ROBOT_STATE_DOCKING) {
		mr_behav->onUpdateGoal(poseHome);
	}	
}

void MR_State::on_cmd_calc_route(const nav_msgs::Odometry& odom_msg, const geometry_msgs::PoseStamped& goal)
{
}


void MR_State::on_cmd_launch()
{
    curr_state = ROBOT_STATE_IDLE;
    threadRunning = true;
    threadMain = std::thread(&MR_State::threadFunction, this);
}


void MR_State::on_cmd_reset()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);
    switch_behav(ROBOT_STATE_IDLE);
    curr_region = 0;
}


void MR_State::on_cmd_pause()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);

    if (curr_state == ROBOT_STATE_DOCKING ||
        curr_state == ROBOT_STATE_GOINGHOME  ||
        curr_state == ROBOT_STATE_TRANSFERRING  ||
        curr_state == ROBOT_STATE_WORKING ||
        curr_state == ROBOT_STATE_STRUGGLING)
    {
        pause_behav();
    }
}

void MR_State::on_cmd_start()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);

    if (!map_initialized) {
        return;
    }
    
    if (curr_state == ROBOT_STATE_PAUSED) {
        resume_behav();
    }
    else if (curr_state == ROBOT_STATE_IDLE) {
        if (has_gnss_fixed()) {
            switch_behav(ROBOT_STATE_TRANSFERRING);
        }
        else {
            state_before_locate = ROBOT_STATE_TRANSFERRING;
            switch_behav(ROBOT_STATE_LOCATING);
        }
    }
    else if (curr_state == ROBOT_STATE_STUCK) {
        switch_behav(state_before_stuck);
    }
}


void MR_State::on_cmd_gohome()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);

    if (!map_initialized) {
        return;
    }
    
    if (curr_state == ROBOT_STATE_WORKING ||
        curr_state == ROBOT_STATE_TRANSFERRING ||
        curr_state == ROBOT_STATE_STRUGGLING ||
        curr_state == ROBOT_STATE_IDLE)
    {
        switch_behav(ROBOT_STATE_GOINGHOME);
    }
    else if (curr_state == ROBOT_STATE_PAUSED) {
        if (state_before_pause == ROBOT_STATE_GOINGHOME ) {
            resume_behav();
        } 
        else if (state_before_pause == ROBOT_STATE_TRANSFERRING ||
                 state_before_pause == ROBOT_STATE_WORKING ||
                 state_before_pause == ROBOT_STATE_STRUGGLING)
        {
            switch_behav(ROBOT_STATE_GOINGHOME);
        }
    } 
}

void MR_State::setObstacleData(const mower_msgs::MowerObstacle& msg)
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
	mr_route_data::getInstance()->setDiscreteObstacle(msg);

}

