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


#ifndef __MR_GENMAP_TOOL_H__
#define __MR_GENMAP_TOOL_H__

#include "map_comm.h"


class mr_genmap_tool 
{
private:

    bool point_in_polygon(mr_point& point, std::vector<mr_point>& vec_point); 
    bool is_intersect(std::vector<mr_point>& list_point1, std::vector<mr_point>& list_point2);
    bool unify_map(mr_raw_map& preprocess_map, mr_mower_map& unified_map);

    void convert_to_local(mr_point& point, mr_local_point& local_point);
    void convert_to_grid(std::vector<signed char>& vecGridData, int width, int height, cv::Mat& image);
    void convert_to_image(mr_region& region,  cv::Mat& image);
    void calc_bound(int& height, int& width, mr_mower_map& unified_map);
public:
    mr_genmap_tool();
    ~mr_genmap_tool();
    bool get_mower_map(mr_raw_map& preprocess_map, mr_mower_map& mower_map);
};


#endif
