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


#ifndef __MR_ROUTEDATA_H__
#define __MR_ROUTEDATA_H__

#include "mr_navi_types.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <costmap_converter/costmap_converter_interface.h>
#include "mower_msgs/MowerObstacle.h"
#include "astar.hpp"

static const double DIFF_BASE_LINK = 0.2;
static const double RADIUS_BASE_LINK = 0.25;

class mr_route_data {
private:
    std::vector<geometry_msgs::PoseStamped> vec_trajectory;
    
	nav_msgs::OccupancyGrid mapData;
	LocalPointHashMap hash_obstacle_point;

	ros::Publisher pub_free_path;

	void convertMap2OdomCoord(const mr_local_point& mapPoint, mr_point& odomPoint);
	void convertOdom2MapCoord(const mr_point& odomPoint, mr_local_point& mapPoint);

	bool detectOccupiedCell(int x, int y);

	void convertToPathMsg(std::vector<mr_point>& vecPath, nav_msgs::Path& path_msg);
	void doOptimize(std::vector<mr_point>& inputPoints);
	void findRoute(mr_point& start, mr_point& end, nav_msgs::Path& pathResult, bool extendValid);
	

	mr_route_data();  

public:
    ~mr_route_data();
    mr_route_data(const mr_route_data&) = delete;
    mr_route_data& operator=(const mr_route_data&) = delete;
    static mr_route_data* getInstance();
	
	bool doRouteCalc(const geometry_msgs::Pose poseStart, const geometry_msgs::Pose poseEnd, 
					 nav_msgs::Path& freePath);

	void setMapData(const nav_msgs::OccupancyGrid& msg);
	void setDiscreteObstacle(const mower_msgs::MowerObstacle& msg);

	void setFreePathPublisher(ros::Publisher& pub);

	bool queryObstacle_odom(mr_point& point);
	bool queryObstacle_map(mr_point& point);

	double minDistanceObstacle(const mr_point& base_center);

};

#endif