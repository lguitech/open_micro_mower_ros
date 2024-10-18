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

#ifndef __MR_LOCATION_H__
#define  __MR_LOCATION_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>


#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include "mr_navi_types.h"
#include "era_types.h"
#include "mr_serialport.h"
#include "mower_msgs/MowerChassisState.h"
#include "mower_msgs/MowerMapMeta.h"
#include "mower_msgs/MowerGnss.h"

#define PARSE_BUFFER_LEN  48

class mr_class_location {
private:
	const double POS_DIFF_DOCK = 1.0;
	
	tf::TransformListener tf_listener;
	
    bool is_simu = false;
    ros::NodeHandle handle;         
    ros::NodeHandle nh_private; 

    ros::Subscriber sub_gnss;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_chassis_status;
    ros::Subscriber sub_mower_map_meta;
	ros::Subscriber sub_aruco_detect;

    ros::Publisher pub_mower_gnss;
    ros::Publisher pub_mower_odom;
	ros::Publisher pub_update_home;


    std::thread threadRead;
    bool threadReadRunning;
    void fun_read_location();
    void start(const char* devname, int baudrate);
    void stop();

    mr_class_serialport m_serialport;

    int recv_serialport_data(unsigned char* buffer, int size); 
    int send_serialport_data(unsigned char* buffer, int size); 

    void cb_gnss(const sensor_msgs::NavSatFix::ConstPtr& navStaFix);
    void cb_imu(const sensor_msgs::Imu::ConstPtr& sonar_msg);
    void cb_chassis_state(const mower_msgs::MowerChassisState::ConstPtr& msg);
    void cb_map_meta(const mower_msgs::MowerMapMeta::ConstPtr& msg);
	void cb_aruco_detect(const geometry_msgs::Pose::ConstPtr& msg);

    mr_point pt_original;
    bool map_initialized = false;

    EraRawGnss raw_gnss{};
    EraRawGGA raw_gga{};
    EraRawIMU raw_imu{};
    unsigned char parse_buffer[PARSE_BUFFER_LEN]{};
    int pos_tail = 0;

    unsigned char calc_checksum(const unsigned char* sentence, int len);
    int find_begin_tag(const unsigned char* pData, int len);
    void do_parse();
    void parse_data(unsigned char* pData, int len);

    void convertGnss2OdomCoord(mr_point& gnssPoint, mr_point& odomPoint);
    void do_filter(mower_msgs::MowerGnss& msg, Eigen::Vector3d& vecPose);
    void build_publish_odom(Eigen::Vector3d& vecPose);
    void publish_location(mower_msgs::MowerGnss& msg_gnss);
    
    mr_class_location() : nh_private("~") , tf_listener(ros::Duration(5.0))  {}
public:
    static mr_class_location* getInstance() {
        static mr_class_location instance;
        return &instance;
    }

    mr_class_location(const mr_class_location&) = delete;
    mr_class_location& operator=(const mr_class_location&) = delete;

    ~mr_class_location() {};

    void init(bool is_simu);
    void unInit();

};


#endif

