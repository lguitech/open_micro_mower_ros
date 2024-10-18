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


#include "mr_map.h"

#include "mower_msgs/MowerMapMeta.h"
#include "mower_msgs/MowerObstacle.h"
#include "mower_msgs/MowerPath.h"

#include "slic3r_coverage_planner/PlanPath.h"


void mr_class_map::cb_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_odomMsg = *msg;
    publish_local_map();
}


void mr_class_map::cb_map_update(const std_msgs::String::ConstPtr& msg)
{
    std::ifstream file;
    std::string map_content;    
    if (msg == nullptr) {
        file.open(filespecRawMap, std::ios::binary);
    }
    else {
        ROS_INFO_STREAM("cb_map_update " << msg->data);
        file.open(msg->data, std::ios::binary);
    }

    if (file.is_open()) {
        file.seekg(0, std::ios::end);
        std::streampos fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        std::string line;
        while (std::getline(file, line)) {
            map_content += line; 
        }

        file.close();
    }
    else {
        publish_global_empty_map();
        publish_empty_map_meta();

        printf("cb_map_update open file error\r\n");
        return;
    }
    mr_mower_map mower_map;

    try {
        bool ret = mr_mapproc.do_mapproc(map_content, mower_map);
        if (!ret) {
            ROS_ERROR("do map process return error!");
            funErrHandle();
            return;
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR("Caught exception: %s", e.what());
        funErrHandle();
        return;
    }


    publish_global_map(mower_map.map_freeroute);
    publish_map_meta(mower_map);
    publish_path(mower_map);
    publish_channel(mower_map);
}



void mr_class_map::cb_chassis_state(const mower_msgs::MowerChassisState::ConstPtr& msg)
{
    mr_chassis_status chassis_status;

    chassis_status.sonar_front_left = msg->sonar_front_left;
    chassis_status.sonar_front_center = msg->sonar_front_center;
    chassis_status.sonar_front_right = msg->sonar_front_right;

    chassis_status.sonar_rear_left = msg->sonar_rear_left;
    chassis_status.sonar_rear_center = msg->sonar_rear_center;
    chassis_status.sonar_rear_right = msg->sonar_rear_right;


    costmap_converter::ObstacleArrayMsg obstacle_array_msg;
    visualization_msgs::MarkerArray obstacle_marker_array;
    
    mr_cluster.createObstalceObject(chassis_status, obstacle_array_msg, obstacle_marker_array);

    pub_cluster_obstacle.publish(obstacle_array_msg);
    pub_cluster_marker.publish(obstacle_marker_array);

    mr_class_obstacle_convert::getInstance()->updateObstacle(obstacle_array_msg);

    publish_obstacle();
}



void mr_class_map::init(bool is_simu, double detect_threshold, std::string fileSpec, CallbackErrHandle funHandle)
{
    this->is_simu = is_simu;
    this->filespecRawMap = fileSpec;
    this->funErrHandle = funHandle;

    std::string mower_odom_topic;
    std::string mower_map_update_topic;

    std::string mower_global_map_topic;
    std::string mower_map_meta_topic;
    std::string mower_local_map_topic;
    std::string mower_work_path_topic;
    std::string mower_work_channel_topic;
    std::string mower_home_topic;
    
    std::string mower_obstacle_topic;    
    
    std::string mower_chassis_state_topic;
    std::string mower_cluster_obstacle_topic;
    std::string mower_cluster_marker_topic;    

    nh_private.getParam("mower_odom_topic", mower_odom_topic);
    nh_private.getParam("mower_map_update_topic", mower_map_update_topic);

    nh_private.getParam("mower_global_map_topic", mower_global_map_topic);
    nh_private.getParam("mower_map_meta_topic", mower_map_meta_topic);
    nh_private.getParam("mower_local_map_topic", mower_local_map_topic);
    nh_private.getParam("mower_work_path_topic", mower_work_path_topic);
    nh_private.getParam("mower_work_channel_topic", mower_work_channel_topic);
    nh_private.getParam("mower_home_topic", mower_home_topic);
    
    nh_private.getParam("mower_obstacle_topic", mower_obstacle_topic);

    nh_private.getParam("mower_chassis_state_topic", mower_chassis_state_topic);
    nh_private.getParam("mower_cluster_obstacle_topic", mower_cluster_obstacle_topic);
    nh_private.getParam("mower_cluster_marker_topic", mower_cluster_marker_topic);
    

    mr_cluster.set_detect_threshold(detect_threshold);


    if (is_simu) {
        sub_odom = handle.subscribe<nav_msgs::Odometry>(mower_odom_topic, 10, &mr_class_map::cb_odom, this);
    }
    sub_map_update = handle.subscribe<std_msgs::String>(mower_map_update_topic, 10, &mr_class_map::cb_map_update, this);
    sub_chassis_state = handle.subscribe<mower_msgs::MowerChassisState>(mower_chassis_state_topic, 10, &mr_class_map::cb_chassis_state, this);

    pub_global_map = handle.advertise<nav_msgs::OccupancyGrid>(mower_global_map_topic, 10, true);
    pub_map_meta = handle.advertise<mower_msgs::MowerMapMeta>(mower_map_meta_topic, 10, true);
    pub_local_map = handle.advertise<nav_msgs::OccupancyGrid>(mower_local_map_topic, 10, true);
    pub_global_path = handle.advertise<mower_msgs::MowerPath>(mower_work_path_topic, 10, true);
    pub_channel = handle.advertise<nav_msgs::Path>(mower_work_channel_topic, 10, true);
    pub_home = handle.advertise<visualization_msgs::Marker>(mower_home_topic, 10, true);
    pub_obstacle = handle.advertise<mower_msgs::MowerObstacle>(mower_obstacle_topic, 10, true);    
    

    pub_cluster_obstacle = handle.advertise<costmap_converter::ObstacleArrayMsg>(mower_cluster_obstacle_topic, 10);
    pub_cluster_marker = handle.advertise<visualization_msgs::MarkerArray>(mower_cluster_marker_topic, 10);

    pathClient = handle.serviceClient<slic3r_coverage_planner::PlanPath>(
            "slic3r_coverage_planner/plan_path");

    ROS_INFO("Waiting for path server");
    if (!pathClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Path service not found.");
        return;
    }

    ros::Duration(2.0).sleep(); 

    //restore map and release
    cb_map_update(nullptr);
}

void mr_class_map::unInit()
{

}


void mr_class_map::publish_home(geometry_msgs::Point& pt_home)
{
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = "map"; 
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "home";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = pt_home.x;
    marker_msg.pose.position.y = pt_home.y;
    marker_msg.pose.position.z = 0.0;

    marker_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    marker_msg.scale.x = 0.5;  
    marker_msg.scale.y = 0.5;
    marker_msg.scale.z = 0.5;
    marker_msg.color.a = 1.0;  
    marker_msg.color.r = 0.0;  
    marker_msg.color.g = 0.0;  
    marker_msg.color.b = 1.0;  
    pub_home.publish(marker_msg);
}

void mr_class_map::publish_global_empty_map()
{
    nav_msgs::OccupancyGrid map;
    mr_mappub.create_empty_global_map(map);
    mr_class_obstacle_convert::getInstance()->setMapInfo(map.info.resolution, map.info.width, map.info.height);
    pub_global_map.publish(map);
}

void mr_class_map::publish_global_map(mr_grid_map& map_freeroute)
{
    nav_msgs::OccupancyGrid map;
    mr_mappub.create_global_map(PREDEF_RESOLUTION, map_freeroute, map);
    mr_class_obstacle_convert::getInstance()->setMapInfo(map.info.resolution, map.info.width, map.info.height);
    pub_global_map.publish(map);
}

void mr_class_map::publish_empty_map_meta()
{
    mower_msgs::MowerMapMeta msg;
    mr_mappub.create_empty_map_meta(msg);
    pub_map_meta.publish(msg);

    geometry_msgs::Point pt_home;
    pt_home.x = 0;
    pt_home.y = 0;
    publish_home(pt_home);
}

void mr_class_map::publish_map_meta(const mr_mower_map& mower_map)
{
    mower_msgs::MowerMapMeta msg;
    mr_mappub.create_map_meta(mower_map, msg);
    pub_map_meta.publish(msg);

    geometry_msgs::Point pt_home;
    pt_home.x = mower_map.pt_home.x;
    pt_home.y = mower_map.pt_home.y;
    publish_home(pt_home);
}

void mr_class_map::publish_channel(mr_mower_map& mower_map) 
{
    for (int i=0; i<mower_map.list_channel.size(); i++) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (auto& point : mower_map.list_channel[i].list_point) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            path_msg.poses.push_back(pose);
        }
        pub_channel.publish(path_msg);
    }
}


void mr_class_map::publish_path(mr_mower_map& mower_map)
{
    mower_msgs::MowerPath mower_path;

    mr_class_route mr_route;
    mr_route.setPathClient(pathClient);
    mr_route.create_global_route(mower_map, mower_path);
    pub_global_path.publish(mower_path); //...

}


void mr_class_map::publish_local_map()
{
    nav_msgs::OccupancyGrid map;
    mr_mappub.createLocalMap(m_odomMsg.pose.pose.position.x, m_odomMsg.pose.pose.position.y, map);
    pub_local_map.publish(map);
}


void mr_class_map::publish_obstacle()
{
    mower_msgs::MowerObstacle msg;
    mr_mappub.createObstacleMsg(msg);
    pub_obstacle.publish(msg);
}
