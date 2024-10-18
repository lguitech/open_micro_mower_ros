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

#include "mr_routedata.h"
#include <tf/tf.h>
#include "mr_util.h"
#include "mr_teb_controller.h"


mr_route_data::mr_route_data()
{

}

mr_route_data::~mr_route_data()
{

}

mr_route_data* mr_route_data::getInstance()
{
    static mr_route_data instance;
    return &instance;
}

inline void mr_route_data::convertOdom2MapCoord(const mr_point& odomPoint, mr_local_point& mapPoint)
{
    mapPoint.x = static_cast<int>(std::round(odomPoint.x / mapData.info.resolution));
    mapPoint.y = static_cast<int>(std::round(odomPoint.y / mapData.info.resolution));
}

inline void mr_route_data::convertMap2OdomCoord(const mr_local_point& mapPoint, mr_point& odomPoint)
{
    odomPoint.x = (double)mapPoint.x * mapData.info.resolution;
    odomPoint.y = (double)mapPoint.y * mapData.info.resolution;
}


void mr_route_data::setMapData(const nav_msgs::OccupancyGrid& msg)
{
	mapData = msg;
}

bool mr_route_data::detectOccupiedCell(int x, int y)
{
    if (x < 0 || x >= mapData.info.width ||
        y < 0 || y >= mapData.info.height) {
        return true;
    }
    if (mapData.data.at(y * mapData.info.width + x) != MAP_CELL_FREE){
        return true;
    }
    return false;
}


void mr_route_data::setDiscreteObstacle(const mower_msgs::MowerObstacle& msg)
{
	hash_obstacle_point.clear();

	for (const geometry_msgs::Point& point : msg.points) {
		mr_local_point ptMap(point.x, point.y);
		if (!detectOccupiedCell(ptMap.x, ptMap.y)) {
			hash_obstacle_point.insert(ptMap);
		}    
	}
}


void mr_route_data::setFreePathPublisher(ros::Publisher& pub)
{
	pub_free_path = pub;
}




void mr_route_data::convertToPathMsg(std::vector<mr_point>& vecPath, nav_msgs::Path& path_msg)
{
    path_msg.header.frame_id = "map";

    int size = vecPath.size();
    path_msg.poses.clear();
    path_msg.poses.reserve(size);

    for (size_t i = 0; i < size; ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = vecPath[i].x;
        pose_stamped.pose.position.y = vecPath[i].y;

        if (i + 1 < vecPath.size()) {
            double orientation = atan2(vecPath[i + 1].y - vecPath[i].y, 
                                       vecPath[i + 1].x - vecPath[i].x);

            tf::Quaternion quaternion;
            quaternion.setRPY(0, 0, orientation); 
            tf::quaternionTFToMsg(quaternion, pose_stamped.pose.orientation);
        } 
        else {
            double orientation = atan2(vecPath[i - 1].y - vecPath[i - 2].y, 
                                       vecPath[i - 1].x - vecPath[i - 2].x);

            tf::Quaternion quaternion;
            quaternion.setRPY(0, 0, orientation);
            tf::quaternionTFToMsg(quaternion, pose_stamped.pose.orientation);

        }

        pose_stamped.header.stamp = ros::Time::now();
        path_msg.poses.emplace_back(pose_stamped);
    }  
}

void mr_route_data::findRoute(mr_point& start, mr_point& end, nav_msgs::Path& pathResult,
        bool extendValid)
{
    mr_local_point mapPointCurr, mapPointGoal;
    
    convertOdom2MapCoord(start, mapPointGoal); 
    convertOdom2MapCoord(end, mapPointCurr);

    Generator generator;      
	{
		generator.setMapData(mapData.info.width, mapData.info.height, mapData.data);
		generator.setObstacleData(hash_obstacle_point);
	}

    CoordinateList path = generator.findPath(mapPointCurr, mapPointGoal, extendValid);
    if (path.size() == 0) {
		return;
    }

	std::vector<mr_point> vecRoute;
    vecRoute.reserve(path.size());

	for (int i=0; i<path.size(); i++) {
		mr_local_point& mapPoint = path.at(i);
        mr_point odomPoint;
        convertMap2OdomCoord(mapPoint, odomPoint);
        vecRoute.emplace_back(odomPoint);
    }

    if (vecRoute.size() != 0) {
        doOptimize(vecRoute);
    }

	convertToPathMsg(vecRoute, pathResult);
}



mr_point gradientDescent(const mr_point& p1, const mr_point& p2, 
    const mr_point& p3, const mr_point& p4, const mr_point& p5) 
{
    mr_point dp = -1.0 / 6.0 * (p1 - 4 * p2 + 6 * p3 - 4 * p4 + p5);
    mr_point newPoint = p3 + dp;

    return newPoint;
}

void mr_route_data::doOptimize(std::vector<mr_point>& inputPoints)
{
    if (inputPoints.size() < 6) {
        return;
    }
    int iterations = std::min(5, (int)inputPoints.size()- 5);

    for (int iter = 0; iter < iterations; ++iter) {
        for (size_t i = 2; i < inputPoints.size() - 2; ++i) {

            mr_point optimizedPoint = gradientDescent(
                inputPoints[i - 2],
                inputPoints[i - 1],
                inputPoints[i],
                inputPoints[i + 1],
                inputPoints[i + 2]
            );

            inputPoints[i] = optimizedPoint;
        }
    }
}

bool mr_route_data::doRouteCalc(const geometry_msgs::Pose poseStart, const geometry_msgs::Pose poseEnd,
								nav_msgs::Path& freePath)
{
    mr_point startPoint, endPoint;
    
    startPoint.x = poseStart.position.x;
    startPoint.y = poseStart.position.y;

    endPoint.x = poseEnd.position.x;
    endPoint.y = poseEnd.position.y;


    findRoute(startPoint, endPoint, freePath, true);

    if (freePath.poses.size() != 0) {
		ROS_INFO_STREAM("route calc succ!");
		return true;
	}
	else {
		ROS_ERROR_STREAM("route calc fail!");
		return false;
	}
}



bool mr_route_data::queryObstacle_map(mr_point& point)
{
    if (hash_obstacle_point.find(point) == hash_obstacle_point.end()) {
        return false;
    }
    else {
        return true;
    }
}

bool mr_route_data::queryObstacle_odom(mr_point& point)
{
    mr_local_point mapPoint;
    convertOdom2MapCoord(point, mapPoint);
    if (hash_obstacle_point.find(mapPoint) == hash_obstacle_point.end()) {
        return false;
    }
    else {
        return true;
    }
}


double mr_route_data::minDistanceObstacle(const mr_point& base_center)
{
	double min_dis = __DBL_MAX__;
    for (auto it = hash_obstacle_point.begin(); it != hash_obstacle_point.end(); ++it) {
		const mr_local_point& ptMap = *it;
		mr_point ptOdom;
		convertMap2OdomCoord(ptMap, ptOdom);

		double dis = mower::PointDistance(base_center.x, base_center.y, ptOdom.x, ptOdom.y);
		min_dis = std::min(min_dis, dis);
    }        
}	

