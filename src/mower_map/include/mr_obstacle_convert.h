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

#ifndef __MR_OBSTACLE_CONVERT_H__
#define __MR_OBSTACLE_CONVERT_H__

#include "mr_navi_types.h"

#include <mutex>
#include <ros/ros.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>

class mr_class_obstacle_convert {
private:
	const bool INFLATE_OBSTACLE = true;
    double map_resolution;
    int map_width;
    int map_height;

    std::recursive_mutex mutex_obstacle;
    LocalPointHashMap hash_result;

    mr_class_obstacle_convert();

    void fillPolygon(const std::vector<mr_local_point>& polygonEdges); 
    void insertOnePoint(mr_local_point& point);
    void bresenhamLine(mr_local_point start, mr_local_point end) ;
    void convert_to_localpoint(mr_point& point, mr_local_point& local_point);

    void convert_to_pointlist(std::vector<mr_local_point>& vecInput);

    
public:
    
    ~mr_class_obstacle_convert();
    static mr_class_obstacle_convert* getInstance();

    mr_class_obstacle_convert(const mr_class_obstacle_convert&) = delete;
    mr_class_obstacle_convert& operator=(const mr_class_obstacle_convert&) = delete;


    void setMapInfo(double resolution, int width, int height);
    void updateObstacle(costmap_converter::ObstacleArrayMsg& obstacle_array_msg);
    void getAllPoints(std::vector<mr_local_point>& vecPoint);

};
#endif