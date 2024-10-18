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


#include "mr_obstacle_convert.h"

mr_class_obstacle_convert::mr_class_obstacle_convert()
{

}

mr_class_obstacle_convert::~mr_class_obstacle_convert()
{
    
}

mr_class_obstacle_convert* mr_class_obstacle_convert::getInstance() 
{
    static mr_class_obstacle_convert instance;
    return &instance;
}


void mr_class_obstacle_convert::setMapInfo(double resolution, int width, int height)
{
    map_resolution = resolution;
    map_width = width;
    map_height = height;
}


void mr_class_obstacle_convert::insertOnePoint(mr_local_point& point)
{
    if (hash_result.find(point) == hash_result.end()) {
        hash_result.insert(point);
    }
}

bool isHorizontalLine(const mr_local_point& p1, const mr_local_point& p2) {
    return p1.y == p2.y;
}

int interpolateX(const mr_local_point& p1, const mr_local_point& p2, int y) {
    if (p1.y == p2.y) {
        return p1.x;
    }
    return p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
}

void mr_class_obstacle_convert::fillPolygon(const std::vector<mr_local_point>& polygonEdges) {
    if (polygonEdges.empty()) {
        return;
    }

    int minY = polygonEdges[0].y;
    int maxY = polygonEdges[0].y;
    for (const auto& point : polygonEdges) {
        if (point.y < minY) {
            minY = point.y;
        }
        if (point.y > maxY) {
            maxY = point.y;
        }
    }

    for (int y = minY; y <= maxY; ++y) {
        std::vector<int> intersections;

        for (size_t i = 0; i < polygonEdges.size(); ++i) {
            size_t j = (i + 1) % polygonEdges.size();
            const mr_local_point& p1 = polygonEdges[i];
            const mr_local_point& p2 = polygonEdges[j];

            if (isHorizontalLine(p1, p2) && p1.y == y) {
                int xStart = std::min(p1.x, p2.x);
                int xEnd = std::max(p1.x, p2.x);

                for (int x = xStart; x <= xEnd; ++x) {
                    mr_local_point temp(x,y);
                    insertOnePoint(temp);
                }
            }

            if ((p1.y <= y && p2.y > y) || (p1.y > y && p2.y <= y)) {
                int x = interpolateX(p1, p2, y);
                intersections.push_back(x);
            }
        }

        std::sort(intersections.begin(), intersections.end());


        for (size_t i = 0; i < intersections.size(); i += 2) {
            if (i + 1 < intersections.size()) {
                int xStart = intersections[i];
                int xEnd = intersections[i + 1];

                for (int x = xStart; x <= xEnd; ++x) {
                    mr_local_point temp(x,y);
                    insertOnePoint(temp);
                }
            }
        }
    }
}


void mr_class_obstacle_convert::bresenhamLine(mr_local_point start, mr_local_point end) 
{
    int dx = abs(end.x - start.x);
    int dy = abs(end.y - start.y);
    int sx = (start.x < end.x) ? 1 : -1;
    int sy = (start.y < end.y) ? 1 : -1;

    int err = dx - dy;


    while (true) {
        insertOnePoint(start);

        if (start.x == end.x && start.y == end.y) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            start.x += sx;
        }
        if (e2 < dx) {
            err += dx;
            start.y += sy;
        }
        
    }
}

void mr_class_obstacle_convert::convert_to_pointlist(std::vector<mr_local_point>& vecInput)
{
    if (vecInput.size() == 0) {
        return;
    }
    else if (vecInput.size() == 1) {   
        insertOnePoint(vecInput.at(0));
    }
    else if (vecInput.size() == 2) {
        bresenhamLine(vecInput[0], vecInput[1]);
    }
    else  {

        fillPolygon(vecInput);
    }
}

void mr_class_obstacle_convert::convert_to_localpoint(mr_point& point, mr_local_point& local_point)
{
    local_point.x = point.x / PREDEF_RESOLUTION;
    local_point.y = point.y / PREDEF_RESOLUTION;
}


void mr_class_obstacle_convert::updateObstacle(costmap_converter::ObstacleArrayMsg& obstacle_array_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_obstacle);

    hash_result.clear();

    for (size_t i = 0; i < obstacle_array_msg.obstacles.size(); ++i) {
        const costmap_converter::ObstacleMsg& obstacle = obstacle_array_msg.obstacles.at(i);
        
        std::vector<mr_local_point> vec_boundary;        

        for (size_t j=0; j<obstacle.polygon.points.size(); j++) {
            const geometry_msgs::Point32& ptIter = obstacle.polygon.points.at(j);

            mr_point point(ptIter.x, ptIter.y);
            mr_local_point local_point;
            convert_to_localpoint(point, local_point);
            if (vec_boundary.size() == 0) {
                vec_boundary.push_back(local_point);
            } 
            else if (vec_boundary.back() != local_point) { 
                vec_boundary.push_back(local_point);
            }
        }
        
        convert_to_pointlist(vec_boundary);
    }

}

static std::vector<mr_local_point> direction = {
    { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 },
    { 0, 1 }, { -1, 0 }, { 0, -1 }, { 1, 0 },
};

void mr_class_obstacle_convert::getAllPoints(std::vector<mr_local_point>& vecPoint)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_obstacle);
    
    LocalPointHashMap hash_inflat;

	if (INFLATE_OBSTACLE) {
		for (LocalPointHashMap::iterator it = hash_result.begin(); it != hash_result.end(); ++it) {
			const mr_local_point& ptIter = *it;
			for (mr_local_point& offset : direction) {
				mr_local_point pointOffset = ptIter + offset;
				auto iterator = hash_result.find(pointOffset);
				if (iterator == hash_result.end()) {
					iterator = hash_inflat.find(pointOffset);
					if (iterator == hash_inflat.end()) {
						hash_inflat.insert(pointOffset);
					}
				}
			}
		}
	}

    for (LocalPointHashMap::iterator it = hash_result.begin(); it != hash_result.end(); ++it) {
        vecPoint.push_back(*it);
    }

	if (INFLATE_OBSTACLE) {
		for (LocalPointHashMap::iterator it = hash_inflat.begin(); it != hash_inflat.end(); ++it) {
			vecPoint.push_back(*it);
		}
	}

}
