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

#include "mr_route.h"
#include <ros/ros.h>
#include "mr_util.h"
#include "slic3r_coverage_planner/PlanPath.h"

mr_class_route::mr_class_route()
{

}

mr_class_route::~mr_class_route()
{

}

void mr_class_route::setPathClient(ros::ServiceClient& pathClient)
{
    this->pathClient = pathClient;
}

bool mr_class_route::point_in_polygon(mr_point& point, std::vector<mr_point>& vec_point) 
{
    std::vector<cv::Point2f> list_point;
    for (int i=0; i<vec_point.size(); i++) {
        mr_point& pt_curr = vec_point.at(i);
        cv::Point2f point(pt_curr.x, pt_curr.y);
        list_point.push_back(point);
    }
    cv::Point2f pt_param(point.x, point.y);
    double res = pointPolygonTest(list_point, pt_param, true);
    if (res > 0) {
        return true;
    }
    else {
        return false;
    }
}

std::vector<mr_point> mr_class_route::generateCirclePoints(const mr_point& center, double radius, double resolution) 
{
    std::vector<mr_point> points;

    double circumference = 2 * M_PI * radius;

    int numPoints = std::ceil(circumference / resolution);

    double angleIncrement = 2 * M_PI / numPoints;

    for (int i = 0; i < numPoints; ++i) {
        double angle = i * angleIncrement;
        mr_point p;
        p.x = center.x + radius * std::cos(angle);
        p.y = center.y + radius * std::sin(angle);
        points.push_back(p);
    }

    return points;
}

void mr_class_route::create_global_route(mr_mower_map& mower_map, mower_msgs::MowerPath& mower_path)
{
	std::vector<mr_point> home_bound = generateCirclePoints(
			mower_map.pt_home, 1.0, PREDEF_RESOLUTION);

	geometry_msgs::Polygon home_hole;
	for (int i=0; i<home_bound.size(); i++) {
		mr_point& point = home_bound[i];
		geometry_msgs::Point32 pt_hole;
		pt_hole.x = point.x;
		pt_hole.y = point.y;
		pt_hole.z = 0.0;
		home_hole.points.push_back(pt_hole);
	}	

    mower_path.header.frame_id = "map";
    mower_path.header.stamp = ros::Time::now();

    for (auto& region : mower_map.list_region) {    
        slic3r_coverage_planner::PlanPath pathSrv;
        geometry_msgs::Polygon bound;
        std::vector<geometry_msgs::Polygon> holes;

        for (int i=0; i<region.obj_boundary.list_point.size(); i++) {
            mr_point& point = region.obj_boundary.list_point[i];
            geometry_msgs::Point32 pt_bound;
            pt_bound.x = point.x; 
            pt_bound.y = point.y;
            pt_bound.z = 0.0;
            bound.points.push_back(pt_bound);
        }

        for (int i=0; i<region.list_obstacle.size(); i++) {
            mr_object& obstacle = region.list_obstacle[i];
            geometry_msgs::Polygon hole;
            for (int j=0; j<obstacle.list_point.size(); j++) {
                mr_point& point = obstacle.list_point[j];
                geometry_msgs::Point32 pt_hole;
                pt_hole.x = point.x;
                pt_hole.y = point.y;
                pt_hole.z = 0.0;
                hole.points.push_back(pt_hole);
            }
            holes.emplace_back(std::move(hole));
        }

		if (point_in_polygon(mower_map.pt_home, region.obj_boundary.list_point)) {
			holes.emplace_back(home_hole);
		}


        pathSrv.request.angle = 0;
        pathSrv.request.outline_count = 1;
        pathSrv.request.outline = bound;
        pathSrv.request.holes = holes;
        pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
        pathSrv.request.distance = PREDEF_RESOLUTION;     

        if (!pathClient.call(pathSrv)) {
            ROS_ERROR_STREAM("Error getting path area");
            return;
        }   


		std::vector<geometry_msgs::PoseStamped> vecPose; 
		
        for (const auto path : pathSrv.response.paths) {
			vecPose.insert(vecPose.end(), path.path.poses.begin(), path.path.poses.end());
        }   
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

		interpolate(vecPose, path_msg.poses);

        mower_path.paths.emplace_back(path_msg);
    }
}

void mr_class_route::interpolate(const std::vector<geometry_msgs::PoseStamped>& listInput, 
        std::vector<geometry_msgs::PoseStamped>& listOutput)
{
    
    geometry_msgs::PoseStamped pose_this = listInput.at(0);
    listOutput.push_back(pose_this);

    int size = listInput.size();
    for (int i=1; i<size; i++) {
        const geometry_msgs::PoseStamped pose_next = listInput.at(i);
        while(true) {
            double dis = mower::PointDistance(pose_this.pose.position.x, pose_this.pose.position.y, 
                                              pose_next.pose.position.x, pose_next.pose.position.y);
            if (dis > PREDEF_RESOLUTION) {
                double ratio = PREDEF_RESOLUTION/dis;
                double x = ratio * (pose_next.pose.position.x - pose_this.pose.position.x) + pose_this.pose.position.x;
                double y = ratio * (pose_next.pose.position.y - pose_this.pose.position.y) + pose_this.pose.position.y;
                
                pose_this.pose.position.x = x;
                pose_this.pose.position.y = y;
                
                listOutput.push_back(pose_this);
            }
            else {
                pose_this.pose.position.x = pose_next.pose.position.x;
                pose_this.pose.position.y = pose_next.pose.position.y;
                pose_this.pose.orientation = pose_next.pose.orientation;
                listOutput.push_back(pose_this);
                break;
            }
        }
    }
}
