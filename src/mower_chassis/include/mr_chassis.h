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

#ifndef __MR_CHASSIS_H__
#define __MR_CHASSIS_H__

#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include "mr_navi_types.h"
#include "mr_sonarfilter.h"
#include "mower_msgs/MowerChassisControl.h"

#include "mr_serialport.h"

class mr_class_chassis {
private:
    bool is_simu = false;
    double wheel_distance;
    double wheel_radius;
    ros::NodeHandle handle;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_sonar_front_left;
    ros::Subscriber sub_sonar_front_center;
    ros::Subscriber sub_sonar_front_right;
    ros::Subscriber sub_sonar_rear_left;
    ros::Subscriber sub_sonar_rear_center;
    ros::Subscriber sub_sonar_rear_right;
    ros::Subscriber sub_joint_states;
    
    ros::Subscriber sub_chassisControl;
    ros::Subscriber sub_cmd_vel;

    ros::Publisher pub_cmd_vel;

    mr_class_sonar_filter filter_front_left;
    mr_class_sonar_filter filter_front_center;
    mr_class_sonar_filter filter_front_right;

    mr_class_sonar_filter filter_rear_left;
    mr_class_sonar_filter filter_rear_center;
    mr_class_sonar_filter filter_rear_right;

    
    mr_class_serialport m_serialport;

    mr_chassis_status m_value_status;
    mr_chassis_control m_value_control;

    sem_t sem_control;
    std::recursive_mutex mutex_status_data;
    std::recursive_mutex mutex_control_data;
    

    void init_control_value();
    void init_status_value();

    void set_chassis_status(mr_chassis_status* pvalue);

    int recv_serialport_data(char* buffer, int size); 
    int send_serialport_data(char* buffer, int size); 

    float convertSonar(const char* input);

    std::thread threadRead;
    bool threadReadRunning;
    void fun_chassis_read();

    std::thread threadWrite;
    bool threadWriteRunning;
    void fun_chassis_write();

    void startThreadRead(const char* devname, int baudrate);
    void startThreadWrite();
    void stopThreadRead();
    void stopThreadWrite();

    void cb_sonar_front_left(const sensor_msgs::Range::ConstPtr& sonar_msg);
    void cb_sonar_front_center(const sensor_msgs::Range::ConstPtr& sonar_msg);
    void cb_sonar_front_right(const sensor_msgs::Range::ConstPtr& sonar_msg);
    void cb_sonar_rear_left(const sensor_msgs::Range::ConstPtr& sonar_msg);
    void cb_sonar_rear_center(const sensor_msgs::Range::ConstPtr& sonar_msg);
    void cb_sonar_rear_right(const sensor_msgs::Range::ConstPtr& sonar_msg);
    void cb_joint_states(const sensor_msgs::JointState::ConstPtr& joint_states_msg);

    void cb_control(const mower_msgs::MowerChassisControl::ConstPtr& control_msg);
    void cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& control_msg);

    void set_cutter_status(unsigned char value);
    void set_putter_status(unsigned char value);
    void set_brake_status(unsigned char value);

    void set_control_speed(float left_speed, float right_speed);
    void set_control_putter(int status);
    void set_control_cutter(int status);
    void set_control_brake(int status);
    void set_control_system_st(char* sz_status);
    
    void get_chassis_control(mr_chassis_control* pvalue);

    int checkCS(const char *bytes);
    void getAccuracy(float input, char* output, int accuracy = 2, int bufsize = 7);
    void assemble_control_cmd(mr_chassis_control* pvalue, char* sz_cmd);

    void publish_cmd_vel(float left_speed, float right_speed);


    mr_class_chassis() : nh_private("~"){ sem_init(&sem_control, 0, 0); }
public:

    static mr_class_chassis* getInstance() {
        static mr_class_chassis instance;
        return &instance;
    }


    mr_class_chassis(const mr_class_chassis&) = delete;
    mr_class_chassis& operator=(const mr_class_chassis&) = delete;

    void init(bool is_simu, double wheel_distance, double wheel_radius);
    void unInit();
    void get_chassis_status(mr_chassis_status* pvalue);  

};

#endif

