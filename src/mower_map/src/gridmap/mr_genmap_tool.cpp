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
#include "mr_genmap_tool.h"

mr_genmap_tool::mr_genmap_tool()
{


}

mr_genmap_tool::~mr_genmap_tool()
{

}

bool mr_genmap_tool::point_in_polygon(mr_point& point, std::vector<mr_point>& vec_point) 
{
    std::vector<cv::Point2f> list_point;
    for (int i=0; i<vec_point.size(); i++) {
        mr_point& pt_curr = vec_point.at(i);
        cv::Point2f point(pt_curr.x, pt_curr.y);
        list_point.push_back(point);
    }
    cv::Point2f pt_param(point.x, point.y);
    double res = pointPolygonTest(list_point, pt_param, true);
    if (res > 0) {
        return true;
    }
    else {
        return false;
    }
}

bool mr_genmap_tool::is_intersect(std::vector<mr_point>& list_point1, std::vector<mr_point>& list_point2)
{
    size_t num1 = list_point1.size();
    for (size_t i=0; i<num1; i++) {
        mr_point& point = list_point1[i];
        bool ret = point_in_polygon(point, list_point2);
        if (ret) {
            return true;
        }
    }
    return false;
}

bool mr_genmap_tool::unify_map(mr_raw_map& preprocess_map, mr_mower_map& unified_map)
{
    unified_map.pt_offset = preprocess_map.pt_min;
    
    std::vector<mr_object*> list_boundary;
    std::vector<mr_object*> list_obstacle;
    std::vector<mr_object*> list_channel;

    bool has_home = false;

    //先生成三个指针列表
    for (mr_object& object : preprocess_map.list_object) {    
        if (object.type == OBJ_TYPE_ID_BOUNDARY) {
            list_boundary.push_back(&object);
        }
        else if (object.type == OBJ_TYPE_ID_CHANNEL) {
            list_channel.push_back(&object);
        }
        else if (object.type == OBJ_TYPE_ID_OBSTACLE) {
            list_obstacle.push_back(&object);
        }
        else if (object.type == OBJ_TYPE_ID_HOME) {
            unified_map.pt_home = object.list_point[0];
            has_home = true;
        }
    }
    if (!has_home) {
        mr_point& first_point = list_boundary[0]->list_point[0];
        unified_map.pt_home = first_point;
    }


    for (mr_object* boundary : list_boundary) {
        mr_region region;
        region.obj_boundary.type = OBJ_TYPE_ID_BOUNDARY;
        region.obj_boundary.list_point = boundary->list_point;
        for (mr_object* obstacle : list_obstacle) {
            if (is_intersect(obstacle->list_point, region.obj_boundary.list_point)) {
                region.list_obstacle.emplace_back(*obstacle);
            }
        }
        unified_map.list_region.emplace_back(std::move(region));
    }

    for (mr_object* channel : list_channel) {
        unified_map.list_channel.emplace_back(*channel);
    }
    
    return true;
}



void mr_genmap_tool::convert_to_grid(std::vector<signed char>& vecGridData, int width, int height, cv::Mat& image)
{


    for (int y=0; y<height; y++) {
        unsigned char *data = image.data + y * width;
        for (int x=0; x<width; x++) {
            unsigned char value =*(data + x);
            int pos =  y * width + x;   

            if (value == 255) {
                vecGridData.at(pos) = MAP_CELL_FREE;
            }
            else {
                vecGridData.at(pos) = MAP_CELL_OCCUPIED;
            }
        }
    }
}

void mr_genmap_tool::convert_to_local(mr_point& point, mr_local_point& local_point)
{
    local_point.x = point.x / PREDEF_RESOLUTION;
    local_point.y = point.y / PREDEF_RESOLUTION;
}
void mr_genmap_tool::convert_to_image(mr_region& region,  cv::Mat& image)
{
    std::vector<std::vector<cv::Point>> vec_vec_point;
    std::vector<cv::Point> vec_point_boundary;

    for (mr_point& point : region.obj_boundary.list_point) {    
        mr_local_point local_point;
        convert_to_local(point, local_point);
        vec_point_boundary.push_back(local_point);
    }
    vec_vec_point.emplace_back(std::move(vec_point_boundary));

    for (mr_object& obstacle : region.list_obstacle) {    
        std::vector<cv::Point> vec_point_obstacle;
        std::vector<mr_point>& list_point = obstacle.list_point;
        for (mr_point& point : list_point) {
            mr_local_point local_point;
            convert_to_local(point, local_point);
            vec_point_obstacle.push_back(local_point);
        }
        vec_vec_point.emplace_back(std::move(vec_point_obstacle));
    }

    fillPoly(image, vec_vec_point, cv::Scalar(255, 255, 255), cv::LINE_8, 0);  
}

void mr_genmap_tool::calc_bound(int& height, int& width, mr_mower_map& unified_map)
{
    double maxx = 0;
    double maxy = 0;
    //region
    for (mr_region& region : unified_map.list_region) {
        for (mr_point& point : region.obj_boundary.list_point) {
            maxx = std::fmax(maxx, point.x);
            maxy = std::fmax(maxy, point.y);
        }
    }
    for (mr_object& channel : unified_map.list_channel) {
        for (mr_point& point : channel.list_point) {
            maxx = std::fmax(maxx, point.x);
            maxy = std::fmax(maxy, point.y);
        }
    }

    maxx += 2.0;
    maxy += 2.0;
    
    width = maxx / PREDEF_RESOLUTION + 1;
    height = maxy / PREDEF_RESOLUTION + 1;
}


bool mr_genmap_tool::get_mower_map(mr_raw_map& preprocess_map, mr_mower_map& mower_map)
{
    unify_map(preprocess_map, mower_map);

    int height;
    int width;
    calc_bound(height, width, mower_map);
    cv::Mat image_out = cv::Mat::zeros(height, width, CV_8UC1);

    for (mr_region& region : mower_map.list_region) {    
        cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);
        convert_to_image(region, image);
        image_out += image;
    }

    for (mr_object& channel : mower_map.list_channel) {    
        std::vector<std::vector<cv::Point>> vec_vec_point;
        std::vector<cv::Point> vec_point;
        for (mr_point& point : channel.list_point) {
            mr_local_point local_point;
            convert_to_local(point, local_point);
            vec_point.push_back(local_point);
        }
        vec_vec_point.emplace_back(std::move(vec_point));
        polylines(image_out, vec_vec_point, false, cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0);
    }

    mower_map.map_freeroute.width = width;
    mower_map.map_freeroute.height = height;
    mower_map.map_freeroute.vecGrid = std::vector<signed char> (width * height);

    convert_to_grid(mower_map.map_freeroute.vecGrid, width, height, image_out);

    return true;
}

