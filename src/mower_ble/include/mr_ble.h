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

#ifndef __MR_BLE_H__
#define __MR_BLE_H__

#include <ros/ros.h>
#include <mutex>
#include "mr_navi_types.h"
#include "mr_serialport.h"
#include "mower_msgs/MowerGnss.h"
#include "mower_msgs/MowerChassisState.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"


class mr_class_ble {
private:
    double wheel_distance;
    double max_linear_speed;
    ros::NodeHandle handle;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_gnss;
    ros::Subscriber sub_chassis_state;
    ros::Subscriber sub_work_state;

    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_work_control;
    ros::Publisher pub_map_update;

    void cb_gnss(const mower_msgs::MowerGnss::ConstPtr& msg);
    void cb_chassis_state(const mower_msgs::MowerChassisState::ConstPtr& msg);
    void cb_work_state(const std_msgs::Int32::ConstPtr& msg);

    std::recursive_mutex m_mutex_gnss;
    std::recursive_mutex m_mutex_chassis;

    mr_gnss_status m_gnss_status;
    mr_chassis_status m_chassis_status;
    int robot_state;
    std::string strVersion;

    bool threadRunning = false;
    std::thread threadMain;
    void threadFunction();
    
    mr_class_serialport m_serialport;

    std::string m_mapFilePath;

    void startMainThread(const char* szDevName, int baudrate);
    void stopMainThread();

    void readVersion();

    void calc_index(int num_total, int points_per_pack, int& index_start, int& index_end);    
    int checkCS(const char *bytes);
    void onReceivedData(unsigned char* pAttachData, int len);
    void onBLEReceivedMapUpdate();
    void onBLEReceivedWorkControl(int value);
    void onBLEReceivedChassisControl(int angle, int strength);

    int recv_serialport_data(unsigned char* buffer, int size); 
    int send_serialport_data(unsigned char* buffer, int size); 

    void set_gnss_value(mr_gnss_status& value);
    void get_gnss_value(mr_gnss_status& value);
    void set_chassis_value(mr_chassis_status& value);
    void get_chassis_value(mr_chassis_status& value);
    
    void send_map_request(int index_start, int index_end);
    void send_gnss();
    void send_chassis();
    void send_ota_result(int value);
    int req_ota(const char* ssid, const char* password);

    void steer_convert_manual(int deg, int strength, float& left_speed, float& right_speed);
    void convert_cmd_vel(float left_speed, float right_speed, geometry_msgs::Twist& cmd_vel);
    mr_class_ble() : nh_private("~") {}
public:
    static mr_class_ble* getInstance() {
        static mr_class_ble instance;
        return &instance;
    }

    mr_class_ble(const mr_class_ble&) = delete;
    mr_class_ble& operator=(const mr_class_ble&) = delete;

    void init(double max_linear_speed, double wheel_distance);
    void unInit();
};

#endif




