#ifndef __MR_COSTMAP_CONVERTER_H__
#define __MR_COSTMAP_CONVERTER_H__

#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.h>

class mr_costmap_converter
{
private:
    std::recursive_mutex mutex_state;
    ros::NodeHandle handle;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_map;
    ros::Subscriber sub_localmap;

    ros::Publisher pub_obstacle_teb;
    ros::Publisher pub_obstacle_teb_marker;

    nav_msgs::OccupancyGrid glocal_map;  //本地保存一个全局地图的备份
    costmap_2d::Costmap2D costmap;

    std::string frame_id;
    int occupied_min_value = 100;
    bool costmap_initialized = false;

    void cb_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader;
    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter;


    void publishAsMarker(const std::string& frame_id, 
        const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub);
    void publishAsMarker(const std::string& frame_id, 
        const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub);

public:
    mr_costmap_converter();
    ~mr_costmap_converter(){}

    void init();
    void unInit();
};


#endif