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

#ifndef __MR_TEB_CONTROLLER_H__
#define __MR_TEB_CONTROLLER_H__

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


class mr_teb_controller : public mr_base_controller
{
private:
    int const OBSTACLE_ALL = 0; 
    int const OBSTACLE_STATIC_ONLY = 1;
	int const OBSTACLE_DYNAMIC_ONLY = 2;
    int const OBSTACLE_NONE = 3;

    double const THRESHOLD_ARRIVED_NORMAL = 0.4;
	double const THRESHOLD_ARRIVED_HOME = 1.0;

	bool fail_tag = false;
	int route_type;

    std::recursive_mutex mutex_state;

    ros::Subscriber sub_current_goal;
    ros::Subscriber sub_custom_obst;
    ros::Subscriber sub_cluster_obst;
    

    TebConfig config;
    boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;

    PlannerInterfacePtr planner;
    TebVisualizationPtr visual;
    RobotFootprintModelPtr robot_model;
   
    std::vector<ObstaclePtr> obstacle_static;
    std::vector<ObstaclePtr> obstacle_dynamic;
    std::vector<ObstaclePtr> obst_vector;  


    ViaPointContainer via_points;
    PoseSE2 poseStart;
    PoseSE2 poseGoal;
	PoseSE2 poseFinalGoal;

    void cb_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
    void cb_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
	void cb_clusterObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg);

	void recalGoal();

    void getObstacleVector(std::vector<ObstaclePtr>& result, int tag = 0);

	bool has_arrived();

    std::vector<geometry_msgs::PoseStamped> global_path;

	mr_teb_controller();
	
public:
    ~mr_teb_controller();	
    mr_teb_controller(const mr_teb_controller&) = delete;
    mr_teb_controller& operator=(const mr_teb_controller&) = delete;
    static mr_teb_controller* getInstance();

	void set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance) override;
	
	void skip_to_final();
	
    void set_path(nav_msgs::Path& path, int route_type);
	void reset_path();
    void set_pose(const geometry_msgs::Pose& msg);
    int compute_control_command();	
    bool get_process_info();
};

#endif

