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
#include "mr_preproc_tool.h"
#include "cjsonobject.h"
#include "mr_coord.h"
#include "mr_util.h"


mr_preproc_tool::mr_preproc_tool()
{

}

mr_preproc_tool::~mr_preproc_tool()
{

}

bool mr_preproc_tool::parse_raw_map(const char* buffer, mr_raw_map& raw_map)
{

    neb::CJsonObject json(buffer);

    size_t numObj = json["objectList"].GetArraySize();
    for (size_t i=0; i<numObj; i++) {
        mr_object map_obj;
        neb::CJsonObject jsonMapObject;
        json["objectList"].Get(i, jsonMapObject);
        
        jsonMapObject.Get("objType", map_obj.type);
        
        size_t numPoint = jsonMapObject["listPoint"].GetArraySize();

        for (size_t j=0; j<numPoint; j++) {
            neb::CJsonObject jsonPoint;
            jsonMapObject["listPoint"].Get(j, jsonPoint);

            mr_point pt_wgs84;            
            jsonPoint.Get("x", pt_wgs84.x);
            jsonPoint.Get("y", pt_wgs84.y);
            map_obj.list_point.push_back(pt_wgs84);
        }
        raw_map.list_object.emplace_back(std::move(map_obj));
    }
    return true;
}


bool mr_preproc_tool::pre_process(mr_raw_map& raw_map)
{
    mr_point pt_min;
    pt_min.x = __DBL_MAX__;
    pt_min.y = __DBL_MAX__;

    size_t numObj = raw_map.list_object.size();
    bool first = true;
    for (size_t i=0; i<numObj; i++) {
        mr_object& obj_input = raw_map.list_object[i];
        size_t numPoint = obj_input.list_point.size();
        for (size_t j=0; j<numPoint; j++) {
            mr_point& point = obj_input.list_point[j];
            mr_point pt_utm;
            if (first) {
                MR_Coord::getInstance()->convert_to_utm_reset_param(
                            point.x, point.y, &pt_utm.x, &pt_utm.y);    
                first = false;            
            }
            else {
                MR_Coord::getInstance()->convert_to_utm(point.x, point.y, &pt_utm.x, &pt_utm.y);
            }
            point = pt_utm;

            pt_min.x = std::min (pt_min.x, point.x);
            pt_min.y = std::min (pt_min.y, point.y);
        }
    }
    raw_map.pt_min = pt_min;
    raw_map.pt_min.x -= 2;
    raw_map.pt_min.y -= 2;
    return true;
}

static void convert_one_point(mr_point& pt_world, mr_point& pt_offset,
                                mr_point& pt_local)
{
    pt_local.x = pt_world.x - pt_offset.x;
    pt_local.y = pt_world.y - pt_offset.y;

}

bool mr_preproc_tool::offset_map(mr_raw_map& map_input, mr_raw_map& map_output)
{
    map_output.pt_min = map_input.pt_min;
    
    size_t numObj = map_input.list_object.size();
    for (size_t i=0; i<numObj; i++) {
        mr_object& obj_input = map_input.list_object[i];
        mr_object obj_Output;
        obj_Output.type = obj_input.type;
        size_t numPoint = obj_input.list_point.size();


        mr_point& pt_input_prev = obj_input.list_point[0];
        mr_point pt_output_prev;
        convert_one_point(pt_input_prev, map_output.pt_min, pt_output_prev);
        obj_Output.list_point.push_back(pt_output_prev);

        for (size_t j=1; j<numPoint; j++) {
            mr_point& pt_input_curr = obj_input.list_point[j];
            mr_point pt_output_curr;
            convert_one_point(pt_input_curr, map_output.pt_min, pt_output_curr);
            if (pt_output_curr.x != pt_output_prev.x || pt_output_curr.y != pt_output_prev.y) 
            {
                obj_Output.list_point.push_back(pt_output_curr);        
                pt_output_prev = pt_output_curr;
            }
        }
        map_output.list_object.emplace_back(std::move(obj_Output));
    }

    return true;
}

bool mr_preproc_tool::get_preprocess_map(const char* buffer, mr_raw_map& preprocess_map)
{
    mr_raw_map raw_map;
    bool ret = parse_raw_map(buffer, raw_map);
    if (!ret) {
        return false;
    }
    ROS_INFO("mr_preproc_tool::get_preprocess_map parse_raw_map finished successfully!");
    ret = pre_process(raw_map);
    if (!ret) {
        return false;
    }
    ROS_INFO("mr_preproc_tool::get_preprocess_map pre_process finished successfully!");
    ret = offset_map(raw_map, preprocess_map);
    if (!ret) {
        return false;
    }
    ROS_INFO("mr_preproc_tool::get_preprocess_map convert_local_map finished successfully!");
    return true;
}
