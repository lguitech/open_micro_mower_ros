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
#include "mr_mapproc.h"
#include "mr_preproc_tool.h"
#include "mr_genmap_tool.h"

mr_class_mapproc::mr_class_mapproc()
{

}
mr_class_mapproc::~mr_class_mapproc()
{

}

bool mr_class_mapproc::do_mapproc(std::string& strContent, 
                            mr_mower_map& mower_map)
{
    mr_preproc_tool preproc_tool;
    mr_raw_map preprocess_map;
    bool ret = preproc_tool.get_preprocess_map(strContent.c_str(), preprocess_map);
    if (!ret) {
        ROS_ERROR("get_preprocess_map finished error!");
        return false;
    }
    ROS_INFO("get_preprocess_map finished!");

    mr_genmap_tool genmap_tool;
    ret = genmap_tool.get_mower_map(preprocess_map, mower_map);

    if (!ret) {
        ROS_ERROR_STREAM("get_mower_map return false");
        return false;
    }
    ROS_INFO_STREAM("get_grid_map finished! ");

    return true;
}