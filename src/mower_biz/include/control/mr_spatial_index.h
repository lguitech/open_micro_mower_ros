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

#ifndef __MR_SPACE_INDEX_H__
#define __MR_SPACE_INDEX_H__

#include <vector>
#include "mr_navi_types.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "teb_local_planner/teb_local_planner_ros.h"

using namespace teb_local_planner;

class MR_SpatialIndex {
private:

    ViaPointContainer currPath;

    const double SPATIAL_SEARCH_RADIUS = 2.0;
    const double DEFAULT_GRID_SIZE = 2.0;
    double grid_size;
    std::unordered_map<int, std::vector<int>> m_grid;
    
    int computeGridId(int x, int y) const;
    std::vector<int> search(double min_x, double min_y, double max_x, double max_y) const;
public:
    MR_SpatialIndex();
    ~MR_SpatialIndex();

    void set_path(ViaPointContainer& path);
    void findNearestPoint(mr_point& curr_pos, double& min_distance, int& index_result);
};

#endif