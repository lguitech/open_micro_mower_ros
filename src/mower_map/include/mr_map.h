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

#ifndef __MR_MAP_H__
#define  __MR_MAP_H__
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include "map_comm.h"
#include "mr_mapproc.h"
#include "mr_mappub.h"
#include "mr_route.h"
#include "mr_cluster.h"
#include "mr_obstacle_convert.h"
#include "mower_msgs/MowerChassisState.h"

typedef void (*CallbackErrHandle)();
class mr_class_map {
private:
    bool is_simu = false;
    ros::NodeHandle handle;         
    ros::NodeHandle nh_private; 

    ros::Subscriber sub_odom;
    ros::Subscriber sub_map_update;
    ros::Subscriber sub_chassis_state;

    ros::Publisher pub_global_map;
    ros::Publisher pub_map_meta;
    ros::Publisher pub_local_map;
    ros::Publisher pub_channel;
    ros::Publisher pub_home;
    ros::Publisher pub_obstacle;
    ros::Publisher pub_global_path;

    ros::Publisher pub_cluster_marker;
    ros::Publisher pub_cluster_obstacle;

    ros::ServiceClient pathClient;
    
    void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void cb_map_update(const std_msgs::String::ConstPtr& msg);
    void cb_chassis_state(const mower_msgs::MowerChassisState::ConstPtr& msg);

    void publish_home(geometry_msgs::Point& pt_home);
    void publish_global_empty_map();
    void publish_global_map(mr_grid_map& map_freeroute);
    void publish_empty_map_meta();
    void publish_map_meta(const mr_mower_map& mower_map);
    void publish_channel(mr_mower_map& mower_map);
    void publish_path(mr_mower_map& mower_map);
    void publish_local_map();
    void publish_obstacle();


    mr_class_mapproc mr_mapproc;
    mr_class_mappub  mr_mappub;
    
    mr_class_cluster mr_cluster;


    nav_msgs::Odometry m_odomMsg;
    std::string filespecRawMap;

    CallbackErrHandle funErrHandle;
    mr_class_map() : nh_private("~") {}
public:

    static mr_class_map* getInstance() {
        static mr_class_map instance;
        return &instance;
    }

    mr_class_map(const mr_class_map&) = delete;
    mr_class_map& operator=(const mr_class_map&) = delete;

    ~mr_class_map() {};

    void init(bool is_simu, double detect_threshold, std::string fileSpec, CallbackErrHandle funHandle);
    void unInit();

};


#endif

