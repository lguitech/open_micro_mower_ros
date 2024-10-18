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

#ifndef __MAPPROC_COMM_H__
#define  __MAPPROC_COMM_H__

#include "mr_navi_types.h"


#define REGION_INDEX_FOR_HOME 1024

#define OBJ_TYPE_ID_BOUNDARY 1001  
#define OBJ_TYPE_ID_CHANNEL 1002   
#define OBJ_TYPE_ID_OBSTACLE 1003  
#define OBJ_TYPE_ID_PATH 1004      
#define OBJ_TYPE_ID_WORK 1005      
#define OBJ_TYPE_ID_HOME 1006      


struct _mr_object 
{
    int type;
    std::vector<mr_point> list_point;
};
typedef struct _mr_object mr_object;

struct _mr_raw_map 
{
    mr_point pt_min;
    std::vector<mr_object> list_object;
};
typedef struct _mr_raw_map mr_raw_map;


struct _mr_region
{
    mr_object obj_boundary;
    std::vector<mr_object> list_obstacle;
};
typedef struct _mr_region mr_region;


struct _mr_grid_map 
{
    int width;
    int height;
    std::vector<signed char> vecGrid;
    _mr_grid_map() : width(0), height(0) {}
    _mr_grid_map(int w, int h, const std::vector<signed char>& grid) : width(w), height(h), vecGrid(grid) { }
};
typedef struct _mr_grid_map mr_grid_map;


struct _mr_mower_map
{
    mr_point pt_offset;
    mr_point pt_home;
    std::vector<mr_region> list_region;
    std::vector<mr_object> list_channel;
    mr_grid_map map_freeroute;
    
};
typedef struct _mr_mower_map mr_mower_map;


#endif