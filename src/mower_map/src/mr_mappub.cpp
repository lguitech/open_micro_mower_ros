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
#include "mr_mappub.h"
#include "mr_coord.h"

mr_class_mappub::mr_class_mappub()
{

}

mr_class_mappub::~mr_class_mappub()
{

}



void mr_class_mappub::create_empty_map_meta(mower_msgs::MowerMapMeta& map_meta)
{
    map_meta.header.frame_id = "map";
    map_meta.header.stamp = ros::Time::now();

    map_meta.resolution = PREDEF_RESOLUTION;
    
    map_meta.home.x = map_meta.home.y = map_meta.home.z = 0;


    double lon = 116.41138829;
    double lat = 40.14618212;
    double utmx, utmy;
    MR_Coord::getInstance()->convert_to_utm_reset_param(lon, lat, &utmx, &utmy);

    map_meta.original.x = utmx;
    map_meta.original.y = utmy;
    map_meta.original.z = 0;
}

void mr_class_mappub::create_map_meta(const mr_mower_map& mower_map, mower_msgs::MowerMapMeta& map_meta)
{
    map_meta.header.frame_id = "map";
    map_meta.header.stamp = ros::Time::now();

    map_meta.resolution = PREDEF_RESOLUTION;

    map_meta.width = mower_map.map_freeroute.width;
    map_meta.height = mower_map.map_freeroute.height;
    
    map_meta.home.x = mower_map.pt_home.x;
    map_meta.home.y = mower_map.pt_home.y;
    map_meta.home.z = 0;

    map_meta.original.x = mower_map.pt_offset.x;
    map_meta.original.y = mower_map.pt_offset.y;
    map_meta.original.z = 0;
}


void mr_class_mappub::create_empty_global_map(nav_msgs::OccupancyGrid& map)
{
    map.header.stamp = ros::Time::now();
    map.header.frame_id = "map";
    map.info.resolution = PREDEF_RESOLUTION;

    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;

    map.info.width = 200;   
    map.info.height = 200;  
    int size = map.info.width * map.info.height;
    for (int i=0; i<size; i++) {
        map.data.push_back(MAP_CELL_FREE);
    }
}

void mr_class_mappub::create_global_map(double resolution, 
    mr_grid_map& region, nav_msgs::OccupancyGrid& map)
{
    map.header.stamp = ros::Time::now();
    map.header.frame_id = "map"; 
    map.info.resolution = resolution;
    
    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;


    map.info.width = region.width;   
    map.info.height = region.height; 


    map.data = region.vecGrid;
}


void mr_class_mappub::convertOdom2MapCoord(mr_point& odomPoint, mr_local_point& mapPoint)
{
    mapPoint.x = static_cast<int>(std::round(odomPoint.x / PREDEF_RESOLUTION));
    mapPoint.y = static_cast<int>(std::round(odomPoint.y / PREDEF_RESOLUTION));
}

void mr_class_mappub::createLocalMap(double posx, double posy, nav_msgs::OccupancyGrid& local_map)
{
    double resolution = PREDEF_RESOLUTION; 
    int local_map_width = 4 * PREDEF_RESOLUTION_INVERSE;  
    int local_map_height = 4 * PREDEF_RESOLUTION_INVERSE; 

    local_map.header.frame_id = "map";
    local_map.info.resolution = resolution;
    local_map.info.width = local_map_width;
    local_map.info.height = local_map_height;

    posx = std::round(posx * PREDEF_RESOLUTION_INVERSE) / PREDEF_RESOLUTION_INVERSE;
    posy = std::round(posy * PREDEF_RESOLUTION_INVERSE) / PREDEF_RESOLUTION_INVERSE;

    local_map.info.origin.position.x = posx - (local_map_width * resolution) / 2;
    local_map.info.origin.position.y = posy - (local_map_height * resolution) / 2;

    std::vector<int8_t> local_map_data(local_map_width * local_map_height, MAP_CELL_FREE); 
    

    std::vector<mr_local_point> vecPoint;
    mr_class_obstacle_convert::getInstance()->getAllPoints(vecPoint);
    for (auto& globalMapPoint : vecPoint) {
        mr_local_point localMapPoint;
       
        localMapPoint.x = globalMapPoint.x - static_cast<int>(std::round(local_map.info.origin.position.x / resolution));
        localMapPoint.y = globalMapPoint.y - static_cast<int>(std::round(local_map.info.origin.position.y / resolution));

        if (localMapPoint.x < 0 || localMapPoint.x >= local_map_width ||
            localMapPoint.y < 0 || localMapPoint.y >= local_map_height) 
        {
                continue;
        }
        local_map_data.at(localMapPoint.y * local_map_width + localMapPoint.x) = MAP_CELL_OCCUPIED;
    }

    local_map.data = std::move(local_map_data);

}


void mr_class_mappub::createObstacleMsg(mower_msgs::MowerObstacle& msg)
{
    std::vector<mr_local_point> vecPoint;
    mr_class_obstacle_convert::getInstance()->getAllPoints(vecPoint);
    for (auto& point : vecPoint) {
        geometry_msgs::Point msgPoint;
        msgPoint.x = point.x;
        msgPoint.y = point.y;
        msg.points.push_back(msgPoint);
    }
}



void mr_class_mappub::getAdjacentCells(const mr_local_point& center, int map_width, int map_height, 
        std::vector<mr_local_point>& adjacentCells) 
{
    int startX = std::max(0, center.x - EDGE_BUFFER);
    int startY = std::max(0, center.y - EDGE_BUFFER);
    int endX = std::min(map_width - 1, center.x + EDGE_BUFFER);
    int endY = std::min(map_height - 1, center.y + EDGE_BUFFER);

    for (int i = startX; i <= endX; ++i) {
        for (int j = startY; j <= endY; ++j) {
            int distance = sqrt(std::pow(center.x - i, 2) + std::pow(center.y - j , 2));
            if (distance <= EDGE_BUFFER) {
                adjacentCells.emplace_back(i, j);
            }
        }
    }
}


void mr_class_mappub::setEdgeBuffer(nav_msgs::OccupancyGrid& map)
{

    for (int x=0; x<map.info.width; x++) {
        for (int y=0; y<map.info.height; y++) {
            int8_t value = map.data.at(y * map.info.width + x);
            if (
                (value == MAP_CELL_OCCUPIED) ||
                (x == 0 || x == map.info.width - 1 || y == 0 || y == map.info.height - 1)
            )
            {
                std::vector<mr_local_point> adjacentCells;
                getAdjacentCells(mr_local_point(x,y), map.info.width, map.info.height, adjacentCells);
                for (int i=0; i<adjacentCells.size(); i++) {
                    mr_local_point& point = adjacentCells.at(i);
                    int location = point.y * map.info.width + point.x;
                    if (map.data.at(location) != MAP_CELL_OCCUPIED) {
                        map.data.at(location) = MAP_CELL_UNKNOWN;
                        
                    }
                }
            }
        }
    }
}
