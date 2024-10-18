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

#ifndef __MR_DOCK_CONTROLLER_H__
#define __MR_DOCK_CONTROLLER_H__

#include <mutex>
#include <semaphore.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "mr_navi_types.h"
#include "mower_msgs/MowerObstacle.h"
#include "mr_spatial_index.h"
#include "teb_local_planner/teb_local_planner_ros.h"

#include "mr_base_controller.h"

using namespace teb_local_planner;



class mr_dock_controller : public mr_base_controller
{
private:

    double const THRESHOLD_ARRIVED_DIST = 0.02;
	double const THRESHOLD_ARRIVED_ANGLE = 5.0 * M_PI / 180.0;

    //sem_t sem_control;
    std::recursive_mutex mutex_state;

    ros::Subscriber sub_current_goal;
    ros::Subscriber sub_custom_obst;

    

    TebConfig config;
    boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;

    PlannerInterfacePtr planner;
    TebVisualizationPtr visual;
    RobotFootprintModelPtr robot_model;
   
    std::vector<ObstaclePtr> obst_vector;  
    ViaPointContainer via_points;
    PoseSE2 poseStart;
    PoseSE2 poseGoal;

    void cb_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);

	bool has_arrived();

    std::vector<geometry_msgs::PoseStamped> global_path;

	mr_dock_controller();
	
public:
    ~mr_dock_controller();	
    mr_dock_controller(const mr_dock_controller&) = delete;
    mr_dock_controller& operator=(const mr_dock_controller&) = delete;
    static mr_dock_controller* getInstance();

	void set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance) override;
	
    void set_path(nav_msgs::Path& path, int route_type);
	void reset_path();
    void set_pose(const geometry_msgs::Pose& msg);
    int compute_control_command();	
    bool get_process_info();
};

#endif

