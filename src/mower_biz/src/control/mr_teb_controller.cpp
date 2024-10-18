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


#include "mr_teb_controller.h"
#include "mr_navi_types.h"
#include "mr_util.h"


mr_teb_controller* mr_teb_controller::getInstance()
{
    static mr_teb_controller instance;
    return &instance;
}

void mr_teb_controller::set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance)
{
	this->max_linear_speed = max_linear_speed;
	this->max_angular_speed = max_angular_speed;
	this->wheel_distance = wheel_distance;
}


void mr_teb_controller::cb_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
    config.reconfigure(reconfig);
}

void mr_teb_controller::cb_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_state);
    obstacle_static.clear();

    
    for (size_t i = 0; i < obst_msg->obstacles.size(); ++i) {
        if (obst_msg->obstacles.at(i).polygon.points.size() == 1) {
            if (obst_msg->obstacles.at(i).radius == 0) {
                obstacle_static.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                                obst_msg->obstacles.at(i).polygon.points.front().y )));
            }
            else {
                obstacle_static.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                                obst_msg->obstacles.at(i).polygon.points.front().y,
                                                                obst_msg->obstacles.at(i).radius )));
            }
        }
        else if (obst_msg->obstacles.at(i).polygon.points.empty()) {
            ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
            continue;
        }
        else {
            PolygonObstacle* polyobst = new PolygonObstacle;
            for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j) {
                polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                            obst_msg->obstacles.at(i).polygon.points[j].y );
            }
            polyobst->finalizePolygon();
            obstacle_static.push_back(ObstaclePtr(polyobst));
        }

        if(!obstacle_static.empty()) {
            obstacle_static.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
        }
    }
}

void mr_teb_controller::cb_clusterObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg)
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
	
    obstacle_dynamic.clear();

    for (size_t i = 0; i < msg->obstacles.size(); ++i) {
        const costmap_converter::ObstacleMsg& obstacle = msg->obstacles.at(i);
        
        if (obstacle.polygon.points.size() < 3) {
            continue;
        }

        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<obstacle.polygon.points.size(); ++j) {
            polyobst->pushBackVertex(obstacle.polygon.points[j].x,
                                     obstacle.polygon.points[j].y );
        }
        polyobst->finalizePolygon();
        obstacle_dynamic.push_back(ObstaclePtr(polyobst));
    }
}


void mr_teb_controller::getObstacleVector(std::vector<ObstaclePtr>& result, int tag)
{
    result.clear();

    if (tag == OBSTACLE_NONE) {
        return;
    }
    int sizeStatic = obstacle_static.size();
    int sizeDynamic = obstacle_dynamic.size();

    if (tag == OBSTACLE_ALL) { 
        if (sizeStatic + sizeDynamic == 0) {
            return;
        }
        result.reserve(sizeStatic + sizeDynamic);
        if (sizeDynamic != 0) {
            result.insert(result.end(), obstacle_dynamic.begin(), obstacle_dynamic.end());
        }
        if (sizeStatic != 0) {
            result.insert(result.end(), obstacle_static.begin(), obstacle_static.end());
        }

    }
    else if (tag == OBSTACLE_STATIC_ONLY) {
        if (sizeStatic == 0) {
            return;
        }
        result.reserve(sizeStatic);
        result.insert(result.end(), obstacle_static.begin(), obstacle_static.end());
    }
	else if (tag == OBSTACLE_DYNAMIC_ONLY) {
		if (sizeDynamic == 0) {
            return;
        }
        result.reserve(sizeDynamic);
        result.insert(result.end(), obstacle_dynamic.begin(), obstacle_dynamic.end());
	}
}


mr_teb_controller::mr_teb_controller() : mr_base_controller()
{
    std::string mower_teb_static_obstacle_topic;
	std::string mower_cluster_obstacle_topic;

    config.loadRosParamFromNodeHandle(nh_private);

    nh_private.param<std::string>("mower_teb_static_obstacle_topic",  
                                  mower_teb_static_obstacle_topic, "/mower/teb_static_obstacle");

	nh_private.param<std::string>("mower_cluster_obstacle_topic",   
								  mower_cluster_obstacle_topic, "/mower/cluster_obstacle");

    sub_custom_obst = handle.subscribe<costmap_converter::ObstacleArrayMsg>(mower_teb_static_obstacle_topic, 10, 
        &mr_teb_controller::cb_customObstacle, this);

	sub_cluster_obst = handle.subscribe<costmap_converter::ObstacleArrayMsg>(mower_cluster_obstacle_topic, 10, 
		&mr_teb_controller::cb_clusterObstacle, this);

    visual = TebVisualizationPtr(new TebVisualization(nh_private, config));
    robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(nh_private, config);

    if (config.hcp.enable_homotopy_class_planning) {
        planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
    }
    else {
        planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
    }
    
}



mr_teb_controller::~mr_teb_controller()
{

}

	
void mr_teb_controller::skip_to_final()
{
	isFinished = true;
}

void mr_teb_controller::set_pose(const geometry_msgs::Pose& msg) 
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
    double yaw = tf2::getYaw(msg.orientation);
    poseStart = {msg.position.x, msg.position.y, yaw};
}


void mr_teb_controller::set_path(nav_msgs::Path& path, int route_type)
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
    global_path = path.poses;

	geometry_msgs::PoseStamped& finalPose = path.poses.back();
    double thetaFinalGoal = tf::getYaw(finalPose.pose.orientation);
    poseFinalGoal = {finalPose.pose.position.x, finalPose.pose.position.y, thetaFinalGoal};

    isFinished = false;  

    planner->clearPlanner();

	fail_tag = false;

	this->route_type = route_type;

}

void mr_teb_controller::reset_path()
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
    global_path.clear();
	
}

const double GOAL_DIST_THRESHOLD = 3.0;
const int MIN_DIST_NUMBER = 10; 
void mr_teb_controller::recalGoal()
{
	if (global_path.size() == 0) {
		return;  
	}

	double min_distance = __DBL_MAX__;
	int indexLast = -1;
	int upBound = std::min(MIN_DIST_NUMBER, (int)global_path.size());

	for (int index = 0; index < upBound; index++) {
		geometry_msgs::PoseStamped& ptCurr = global_path.at(index);
		double dist_sq = mower::PointDistanceSquare(poseStart.x(), poseStart.y(), 
													ptCurr.pose.position.x, ptCurr.pose.position.y);
		if (dist_sq < min_distance) {
			min_distance = dist_sq;
			indexLast = index;
		}
	} 

	double sum_dist = 0;

	geometry_msgs::PoseStamped ptLast  = global_path.at(indexLast++);
	
	for(;indexLast<global_path.size(); indexLast++) {
		geometry_msgs::PoseStamped ptCurr = global_path.at(indexLast);
		sum_dist += mower::PointDistance(ptLast.pose.position.x, ptLast.pose.position.y,
											ptCurr.pose.position.x, ptCurr.pose.position.y);
		if (sum_dist > GOAL_DIST_THRESHOLD) {
			break;
		}
	}

	if (indexLast == global_path.size()) {
		indexLast = global_path.size() - 1; 
	}

	geometry_msgs::PoseStamped& ptCurr = global_path.at(indexLast);
	double thetaGoal = tf::getYaw(ptCurr.pose.orientation);
	poseGoal = {ptCurr.pose.position.x, ptCurr.pose.position.y, thetaGoal};				

}

bool mr_teb_controller::has_arrived()
{
    double dis = std::hypot(poseFinalGoal.x() - poseStart.x(), poseFinalGoal.y() - poseStart.y());
	if (route_type == ROUTE_TYPE_GOHOME) {
		if (dis <= THRESHOLD_ARRIVED_HOME) {
			isFinished = true;
		}
    }
	else {
		if (dis < THRESHOLD_ARRIVED_NORMAL) {
			isFinished = true;
		}
	}
	return isFinished;
}

int mr_teb_controller::compute_control_command()
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);

	if (isFinished) {
		stop_robot();
		return 0;
	}

    geometry_msgs::Twist cmd_vel;    

    if (global_path.size() == 0) {
        stop_robot();
        return 0;
    }

	if (has_arrived()) {
		stop_robot();
		return 0;
	}

	recalGoal();

    double vx, vy, omega;
	if (fail_tag) {
		getObstacleVector(obst_vector, OBSTACLE_DYNAMIC_ONLY);	
		planner->clearPlanner();
	}
	else {
		getObstacleVector(obst_vector, OBSTACLE_ALL);	
	}

	bool result = planner->plan(poseStart, poseGoal);
	planner->visualize();
	bool ret = planner->getVelocityCommand(vx, vy, omega, config.trajectory.control_look_ahead_poses);    
	if (!ret) {
		fail_tag = true;
	}

	cmd_vel.linear.x = vx;
	cmd_vel.angular.z = omega;  
	pub_cmd_vel.publish(cmd_vel);

	return 0;
}

bool mr_teb_controller::get_process_info()
{
	std::lock_guard<std::recursive_mutex> lock(mutex_state);
    return isFinished;
}

