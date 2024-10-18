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

#ifndef __MR_CLUSTER_H__
#define __MR_CLUSTER_H__

#include "mr_navi_types.h"
#include <ros/ros.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>



class mr_class_cluster {
private:
    tf::TransformListener tf_listener;

    const int OBSTACLE_VALID_TIME = 20;  

    const float CLUSTER_DIS_THRESHOLD = 0.5f;
    const int MIN_CLUSTER_SIZE = 3; 
    const int MAX_CLUSTER_SIZE = 500;
	const int COORD_CONVERT_PARAMTER = 50; 
    double detect_threshold;

    int last_marker_number, last_valid_marker_number;

    LocalPointHashMapExt<uint64> hash_obstacle_point;

    void delay_remove_points(uint64 curr_time);
    void add_point(mr_point& point,  uint64 curr_time);

    void clusterAndComputeConvexHulls(std::vector<std::vector<mr_point>>& vec_vec_result);
    void convertAndPublishObstacles(const std::vector<std::vector<mr_point>>& vec_vec_result,
                                    costmap_converter::ObstacleArrayMsg& obstacle_array_msg,
                                    visualization_msgs::MarkerArray& obstacle_marker_array);

    void transformOneLink(std::string frame_id, double distance, mr_point& result);
public:
    mr_class_cluster(double detect_threshold = 1.2);
    ~mr_class_cluster();
    void set_detect_threshold(double value);
    void createObstalceObject(mr_chassis_status& chassis_status,
                        costmap_converter::ObstacleArrayMsg& obstacle_array_msg,
                        visualization_msgs::MarkerArray& obstacle_marker_array);

};
#endif