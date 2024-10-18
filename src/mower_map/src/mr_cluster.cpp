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

#include "mr_cluster.h"

mr_class_cluster::mr_class_cluster(double detect_threshold): tf_listener(ros::Duration(5.0)) 
{
    this->detect_threshold = detect_threshold;
    last_marker_number = last_valid_marker_number = 0;
}

mr_class_cluster::~mr_class_cluster() 
{

}
void mr_class_cluster::set_detect_threshold(double value)
{
    this->detect_threshold = value;
}

void mr_class_cluster::delay_remove_points(uint64 curr_time)
{
    for (auto it = hash_obstacle_point.begin(); it != hash_obstacle_point.end(); ) {
        if (curr_time - it->second > OBSTACLE_VALID_TIME) { 
            it = hash_obstacle_point.erase(it);  
        } 
        else {
            ++it;
        }
    }        
}


void mr_class_cluster::add_point(mr_point& point, uint64 curr_time)
{
    mr_local_point local_point(point.x * COORD_CONVERT_PARAMTER, point.y * COORD_CONVERT_PARAMTER);

    auto iterator = hash_obstacle_point.find(local_point);
    
    if (iterator == hash_obstacle_point.end()) {
        hash_obstacle_point[local_point] = curr_time;
    }
    else {
        iterator->second = curr_time;
    }
}

void mr_class_cluster::clusterAndComputeConvexHulls(std::vector<std::vector<mr_point>>& vec_vec_result) 
{
    vec_vec_result.clear();
    if (hash_obstacle_point.size() == 0) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto it = hash_obstacle_point.begin(); it != hash_obstacle_point.end(); it++) {
        const mr_local_point& localpoint = it->first;
        cloud->points.push_back(pcl::PointXYZ((double)localpoint.x / COORD_CONVERT_PARAMTER, 
											  (double)localpoint.y / COORD_CONVERT_PARAMTER, 0.0));
    }
    

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(CLUSTER_DIS_THRESHOLD);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);    
    ec.setMaxClusterSize(MAX_CLUSTER_SIZE);  
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::ConvexHull<pcl::PointXYZ> chull;
    

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
        chull.setInputCloud(cluster);
        chull.reconstruct(*hull);

        std::vector<mr_point> hull_points;
        for (const auto& point : hull->points) {
            hull_points.emplace_back(point.x, point.y);
        }
        vec_vec_result.emplace_back(std::move(hull_points));
    }
}

void mr_class_cluster::convertAndPublishObstacles(const std::vector<std::vector<mr_point>>& vec_vec_result,
                                    costmap_converter::ObstacleArrayMsg& obstacle_array_msg,
                                    visualization_msgs::MarkerArray& obstacle_marker_array)

{

    for (size_t i = 0; i < vec_vec_result.size(); ++i) {
        const auto& points = vec_vec_result[i];

        costmap_converter::ObstacleMsg obstacle_msg;
        visualization_msgs::Marker marker_msg;

        marker_msg.header.frame_id = "map";
        marker_msg.ns = "obstacle_shapes";
        marker_msg.action = visualization_msgs::Marker::ADD;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
        marker_msg.scale.x = 0.03;  // Line width
        marker_msg.id = i;

        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;
        marker_msg.color.a = 1.0;

        for (const auto& pt : points) {
            geometry_msgs::Point32 ros_point32;
            geometry_msgs::Point ros_point;
            ros_point32.x = ros_point.x = pt.x;
            ros_point32.y = ros_point.y = pt.y;
            ros_point32.z = ros_point.z = 0;
            
            obstacle_msg.polygon.points.push_back(ros_point32);
            marker_msg.points.push_back(ros_point);            
        }

        const mr_point& first_point = points.front();
        const mr_point& end_point = points.back();

        if (points.size() > 2 && first_point != end_point) {    
            obstacle_msg.polygon.points.push_back(obstacle_msg.polygon.points.front());
            marker_msg.points.push_back(marker_msg.points.front());
        }

        obstacle_array_msg.obstacles.push_back(obstacle_msg);
        obstacle_marker_array.markers.push_back(marker_msg);
    }

    last_valid_marker_number = obstacle_marker_array.markers.size();

    for (size_t i = last_valid_marker_number; i < last_marker_number; ++i) {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.ns = "obstacle_shapes";
        delete_marker.id = i;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        obstacle_marker_array.markers.push_back(delete_marker);
    }
    last_marker_number = last_valid_marker_number;
}


void mr_class_cluster::transformOneLink(std::string frame_id, double distance, mr_point& result)
{
    tf::StampedTransform transform;
    try {
        tf_listener.waitForTransform("map", frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform("map", frame_id, ros::Time(0), transform);            
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Transform exception: %s", ex.what());
        return;
    }    
    tf::Vector3 ptCenter(distance, 0, 0);
    ptCenter = transform * ptCenter;
    result.x = ptCenter.x();
    result.y = ptCenter.y();

}

void mr_class_cluster::createObstalceObject(mr_chassis_status& chassis_status,
                                    costmap_converter::ObstacleArrayMsg& obstacle_array_msg,
                                    visualization_msgs::MarkerArray& obstacle_marker_array)

{
    uint64 curr_time = ros::Time::now().toSec();

    mr_point result;
    if (chassis_status.sonar_front_left < detect_threshold) {
        transformOneLink("sonar_link_front_left", chassis_status.sonar_front_left, result);
        add_point(result, curr_time);
    }
    if (chassis_status.sonar_front_center < detect_threshold) {
        transformOneLink("sonar_link_front_center", chassis_status.sonar_front_center, result);
        add_point(result, curr_time);
    }
    if (chassis_status.sonar_front_right < detect_threshold) {
        transformOneLink("sonar_link_front_right", chassis_status.sonar_front_right, result);
        add_point(result, curr_time);
    }
   
    if (chassis_status.sonar_rear_left < detect_threshold) {
        transformOneLink("sonar_link_rear_left", chassis_status.sonar_rear_left, result);
        add_point(result, curr_time);
    }
    if (chassis_status.sonar_rear_center < detect_threshold) {
        transformOneLink("sonar_link_rear_center", chassis_status.sonar_rear_center, result);
        add_point(result, curr_time);
    }
    if (chassis_status.sonar_rear_right < detect_threshold) {
        transformOneLink("sonar_link_rear_right", chassis_status.sonar_rear_right, result);
        add_point(result, curr_time);
    }

    delay_remove_points(curr_time);
    std::vector<std::vector<mr_point>> vec_vec_result;
    clusterAndComputeConvexHulls(vec_vec_result);
    convertAndPublishObstacles(vec_vec_result, obstacle_array_msg, obstacle_marker_array);
}
