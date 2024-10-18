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

#ifndef __MR_MAPPUB_H__
#define __MR_MAPPUB_H__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "mower_msgs/MowerMapMeta.h"
#include "mower_msgs/MowerObstacle.h"
#include "mr_obstacle_convert.h"

#include "map_comm.h"

class mr_class_mappub 
{
private:
    const int EDGE_BUFFER = 1; 

    double map_meta_resolution;
    mr_point map_meta_offset;
    int map_meta_width;
    int map_meta_height;

    void setEdgeBuffer(nav_msgs::OccupancyGrid& map);
    void getAdjacentCells(const mr_local_point& center, int map_width, int map_height,
            std::vector<mr_local_point>& adjacentCells); 

    void convertOdom2MapCoord(mr_point& odomPoint, mr_local_point& mapPoint);
public:
    mr_class_mappub();
    ~mr_class_mappub();

    void create_empty_global_map(nav_msgs::OccupancyGrid& map);
    void create_empty_map_meta(mower_msgs::MowerMapMeta& map_meta);

    void create_global_map(double resolution, mr_grid_map& map_freeroute, nav_msgs::OccupancyGrid& map);
    void create_map_meta(const mr_mower_map& mower_map, mower_msgs::MowerMapMeta& map_meta);

    void createLocalMap(double posx, double posy, nav_msgs::OccupancyGrid& local_map);

    void createObstacleMsg(mower_msgs::MowerObstacle& msg);
};


#endif
