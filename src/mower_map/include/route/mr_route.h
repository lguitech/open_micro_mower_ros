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

#ifndef __MR_ROUTE_H__
#define __MR_ROUTE_H__

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include "map_comm.h"
#include "mower_msgs/MowerPath.h"

class mr_class_route 
{
private:
    ros::ServiceClient pathClient;

    void interpolate(const std::vector<geometry_msgs::PoseStamped>& listInput, 
        std::vector<geometry_msgs::PoseStamped>& listOutput);
	bool point_in_polygon(mr_point& point, std::vector<mr_point>& vec_point);
	std::vector<mr_point> generateCirclePoints(const mr_point& center, double radius, double resolution);
public:
    mr_class_route();
    ~mr_class_route();
    
    void setPathClient(ros::ServiceClient& pathClient);
    void create_global_route(mr_mower_map& mower_map, mower_msgs::MowerPath& mower_path);

};


#endif
