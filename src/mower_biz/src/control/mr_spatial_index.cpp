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


#include "mr_spatial_index.h"
#include "mr_util.h"

MR_SpatialIndex::MR_SpatialIndex()
{
    grid_size = DEFAULT_GRID_SIZE;
}

MR_SpatialIndex::~MR_SpatialIndex()
{

}


void MR_SpatialIndex::set_path(ViaPointContainer& via_points) 
{
    currPath = via_points;
    m_grid.clear();
    for (int index=0; index<via_points.size(); index++) {
        const Eigen::Vector2d& point = via_points.at(index);
        int grid_x = static_cast<int>(floor(point.x() / grid_size));
        int grid_y = static_cast<int>(floor(point.y() / grid_size));
        int grid_id = computeGridId(grid_x, grid_y);
        m_grid[grid_id].push_back(index);
    }
}

std::vector<int> MR_SpatialIndex::search(
    double min_x, double min_y, double max_x, double max_y) const 
{
    std::vector<int> result;

    int min_grid_x = static_cast<int>(floor(min_x / grid_size));
    int min_grid_y = static_cast<int>(floor(min_y / grid_size));
    int max_grid_x = static_cast<int>(floor(max_x / grid_size));
    int max_grid_y = static_cast<int>(floor(max_y / grid_size));

    for (int grid_x = min_grid_x; grid_x <= max_grid_x; ++grid_x) {
        for (int grid_y = min_grid_y; grid_y <= max_grid_y; ++grid_y) {
            int grid_id = computeGridId(grid_x, grid_y);
            auto it = m_grid.find(grid_id);
            if (it != m_grid.end()) {
                result.insert(result.end(), it->second.begin(), it->second.end());
            }
        }
    }

    return result;
}


int MR_SpatialIndex::computeGridId(int x, int y) const {
    return x * 10000 + y;
}

void MR_SpatialIndex::findNearestPoint(mr_point& curr_pos, double& min_distance, int& index_result)
{
    min_distance = __DBL_MAX__;
    index_result = -1;
    double minx,miny, maxx, maxy;
    minx = std::max(0.0, curr_pos.x - SPATIAL_SEARCH_RADIUS);
    miny = std::max(0.0, curr_pos.y - SPATIAL_SEARCH_RADIUS);

    maxx = curr_pos.x + SPATIAL_SEARCH_RADIUS;
    maxy = curr_pos.y + SPATIAL_SEARCH_RADIUS;

    std::vector<int> index_list = search(minx, miny, maxx, maxy);
    for (int i = 0; i < index_list.size(); i++) {
        int index = index_list.at(i);
        const Eigen::Vector2d &trajPoint = currPath.at(index);
        double dis = mower::PointDistanceSquare(curr_pos.x, curr_pos.y, 
                    trajPoint.x(), trajPoint.y());
        if (dis < min_distance) {
            min_distance = dis;
            index_result = index;
        }
    } 
}