#include <ros/ros.h>
#include "costmap_converter/mr_costmap_converter.h"
#include <visualization_msgs/Marker.h>
#include <costmap_converter/costmap_converter_interface.h>

mr_costmap_converter::mr_costmap_converter():  converter_loader("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), nh_private("~"){
    std::string converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
    nh_private.param("converter_plugin", converter_plugin, converter_plugin);

    try
    {
        converter = converter_loader.createInstance(converter_plugin);
    }
    catch(const pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        ros::shutdown();
    }

    ROS_INFO_STREAM("Standalone costmap converter:" << converter_plugin << " loaded.");        

    std::string mower_map_topic;

    std::string mower_teb_static_obstacle_topic;
    std::string mower_teb_static_obstacle_mark_topic;

    this->nh_private.getParam("mower_map_topic", mower_map_topic);
    this->nh_private.getParam("mower_teb_static_obstacle_topic", mower_teb_static_obstacle_topic);
    this->nh_private.getParam("mower_teb_static_obstacle_mark_topic", mower_teb_static_obstacle_mark_topic);

    sub_map = handle.subscribe(mower_map_topic, 10, &mr_costmap_converter::cb_map, this);

    pub_obstacle_teb = handle.advertise<costmap_converter::ObstacleArrayMsg>(mower_teb_static_obstacle_topic, 10, true);
    pub_obstacle_teb_marker = handle.advertise<visualization_msgs::Marker>(mower_teb_static_obstacle_mark_topic, 10, true);

    if (converter) {
        //converter->setOdomTopic(mower_odom_topic);
        converter->initialize(nh_private);
        converter->setCostmap2D(&costmap);
    }

}
void mr_costmap_converter::init()
{

}

void mr_costmap_converter::unInit()
{

}

void mr_costmap_converter::cb_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    this->glocal_map = *msg;

    costmap.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);

    for (std::size_t i=0; i < msg->data.size(); ++i)
    {
        unsigned int mx, my;
        costmap.indexToCells((unsigned int)i, mx, my);
        costmap.setCost(mx, my, msg->data[i] >= occupied_min_value ? 255 : 0 );
    }

    converter->updateCostmap2D();
    converter->compute();
    costmap_converter::ObstacleArrayConstPtr obstacles = converter->getObstacles();

    if (!obstacles) {
        return;
    }
    pub_obstacle_teb.publish(obstacles);

    frame_id = msg->header.frame_id;

    publishAsMarker(frame_id, *obstacles, pub_obstacle_teb_marker);    
}

void mr_costmap_converter::publishAsMarker(const std::string& frame_id, 
        const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (std::size_t i=0; i<polygonStamped.size(); ++i)
    {
        for (int j=0; j< (int)polygonStamped[i].polygon.points.size()-1; ++j)
        {
			geometry_msgs::Point line_start;
			line_start.x = polygonStamped[i].polygon.points[j].x;
			line_start.y = polygonStamped[i].polygon.points[j].y;
			line_list.points.push_back(line_start);
			geometry_msgs::Point line_end;
			line_end.x = polygonStamped[i].polygon.points[j+1].x;
			line_end.y = polygonStamped[i].polygon.points[j+1].y;
			line_list.points.push_back(line_end);
        }

        if (!polygonStamped[i].polygon.points.empty() && polygonStamped[i].polygon.points.size() != 2 )
        {
			geometry_msgs::Point line_start;
			line_start.x = polygonStamped[i].polygon.points.back().x;
			line_start.y = polygonStamped[i].polygon.points.back().y;
			line_list.points.push_back(line_start);
			if (line_list.points.size() % 2 != 0)
			{
				geometry_msgs::Point line_end;
				line_end.x = polygonStamped[i].polygon.points.front().x;
				line_end.y = polygonStamped[i].polygon.points.front().y;
				line_list.points.push_back(line_end);
			}
        }


    }
    marker_pub.publish(line_list);
}

void mr_costmap_converter::publishAsMarker(const std::string& frame_id, 
        const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.05;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (const costmap_converter::ObstacleMsg& obstacle : obstacles.obstacles)
    {
        for (int j=0; j< (int)obstacle.polygon.points.size()-1; ++j)
        {
            geometry_msgs::Point line_start;
            line_start.x = obstacle.polygon.points[j].x;
            line_start.y = obstacle.polygon.points[j].y;
            line_list.points.push_back(line_start);
            geometry_msgs::Point line_end;
            line_end.x = obstacle.polygon.points[j+1].x;
            line_end.y = obstacle.polygon.points[j+1].y;
            line_list.points.push_back(line_end);
        }

        if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2 )
        {
            geometry_msgs::Point line_start;
            line_start.x = obstacle.polygon.points.back().x;
            line_start.y = obstacle.polygon.points.back().y;
            line_list.points.push_back(line_start);
            if (line_list.points.size() % 2 != 0)
            {
                geometry_msgs::Point line_end;
                line_end.x = obstacle.polygon.points.front().x;
                line_end.y = obstacle.polygon.points.front().y;
                line_list.points.push_back(line_end);
            }
        }
    }
    marker_pub.publish(line_list);
}
