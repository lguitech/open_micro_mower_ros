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
#include <geometry_msgs/Twist.h>
#include "mr_chassis.h"
#include "mr_parsetool.h"
#include "mr_util.h"


int mr_class_chassis::recv_serialport_data(char* buffer, int size)
{
    return m_serialport.recv((unsigned char*)buffer, size);
}

int mr_class_chassis::send_serialport_data(char* buffer, int size)
{
    return m_serialport.send((unsigned char*)buffer, size);
}

void mr_class_chassis::fun_chassis_read()
{
    mr_class_parsetool parse_tool;
    std::vector<std::string> vec_result;

    while(threadReadRunning) {
        usleep(50*1000);

        char buffer[1024];

        int len = recv_serialport_data(buffer, 2048);
        if (len <= 0) {
            continue;
        }
        vec_result.clear();
        parse_tool.append_data(buffer, len, vec_result);
        if (vec_result.size() == 0) {
            continue;
        }

        for (size_t i=0; i<vec_result.size(); i++) {
            std::string& strSentence = vec_result[i];

            std::vector<std::string> vec_split;
            bool ret = mower::split_string(strSentence, vec_split, ',');

            if (ret) {
                mr_chassis_status status;
                int len = vec_split[1].length();
                
                status.left_speed = 
                    atof(vec_split[1].substr(1, len-1).c_str());
                if (strcmp(vec_split[1].substr(0,1).c_str(), "-") == 0) {
                    status.left_speed *= -1;
                }    

                status.right_speed = 
                    atof(vec_split[2].substr(1, len-1).c_str());
                if (strcmp(vec_split[2].substr(0,1).c_str(), "-") == 0) {
                    status.right_speed *= -1;
                }    

                status.putter_state = atoi(vec_split[3].c_str());
                status.cutter_state = atoi(vec_split[4].c_str());
                status.auto_mode = atoi(vec_split[5].c_str());
                status.brake_state = atoi(vec_split[6].c_str());

                status.safe_edge_front = atoi(vec_split[7].substr(0, 1).c_str());
                status.safe_edge_back =  atoi(vec_split[7].substr(1, 1).c_str());
                status.safe_edge_left = atoi(vec_split[7].substr(2, 1).c_str());
                status.safe_edge_right = atoi(vec_split[7].substr(3, 1).c_str());

                status.soc = atoi(vec_split[8].c_str());
                status.battery_volt = atof(vec_split[9].c_str());
                status.battery_charge_state =  atof(vec_split[10].c_str());


                strcpy(status.fault_code, vec_split[11].c_str());
                status.sonar_front_left = 5.0;
                status.sonar_front_right = 5.0;
                
                status.sonar_rear_right = 5.0;
                status.sonar_rear_left = 5.0;

                status.sonar_front_center = 5.0;
                status.sonar_rear_center = 5.0;

                set_chassis_status(&status);
             }
        }
    }
}

void mr_class_chassis::fun_chassis_write()
{
    while(threadWriteRunning) {
        mr_chassis_control control_value;
        get_chassis_control(&control_value);
        char strCmd[256];
        assemble_control_cmd(&control_value, strCmd);
        send_serialport_data(strCmd, strlen(strCmd));
    }
}

float mr_class_chassis::convertSonar(const char* input)
{
    assert(strlen(input) == 3);
    if (
        (input[0]>='0' && input[0]<='9') &&
        (input[1]>='0' && input[1]<='9') &&
        (input[2]>='0' && input[2]<='9')
    )
    {
        return atof(input) / 100;
    }
    else {
        return 5.0;
    }
}

void mr_class_chassis::cb_sonar_front_left(const sensor_msgs::Range::ConstPtr& sonar_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.sonar_front_left = filter_front_left.addData(sonar_msg.get()->range);
}

void mr_class_chassis::cb_sonar_front_center(const sensor_msgs::Range::ConstPtr& sonar_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.sonar_front_center = filter_front_center.addData(sonar_msg.get()->range);
}
void mr_class_chassis::cb_sonar_front_right(const sensor_msgs::Range::ConstPtr& sonar_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.sonar_front_right = filter_front_right.addData(sonar_msg.get()->range);
}

void mr_class_chassis::cb_sonar_rear_left(const sensor_msgs::Range::ConstPtr& sonar_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.sonar_rear_left = filter_rear_left.addData(sonar_msg.get()->range);
}

void mr_class_chassis::cb_sonar_rear_center(const sensor_msgs::Range::ConstPtr& sonar_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.sonar_rear_center = filter_rear_center.addData(sonar_msg.get()->range);
}

void mr_class_chassis::cb_sonar_rear_right(const sensor_msgs::Range::ConstPtr& sonar_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.sonar_rear_right = filter_rear_right.addData(sonar_msg.get()->range);
}

void mr_class_chassis::cb_joint_states(const sensor_msgs::JointState::ConstPtr& joint_states_msg)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    if (joint_states_msg->name.size() != 1) {
        return;
    }
    double rotate_speed = joint_states_msg->velocity[0];
    if (joint_states_msg->name[0] == "left_wheel_joint") {
        m_value_status.left_speed = rotate_speed * wheel_radius;
    }
    else if (joint_states_msg->name[0] == "right_wheel_joint") {
        m_value_status.right_speed = rotate_speed * wheel_radius;
    }
}


void mr_class_chassis::cb_control(const mower_msgs::MowerChassisControl::ConstPtr& control_msg)
{
    mr_chassis_status chassis_status;

    switch (control_msg->control_type)
    {
    case CHASSIS_CONTROL_TYPE_PUTTER:
        set_control_putter(control_msg->param1);
        break;
    case CHASSIS_CONTROL_TYPE_CUTTER:
        set_control_cutter(control_msg->param1);
        break;    
    case CHASSIS_CONTROL_TYPE_BRAKE:
        set_control_brake(control_msg->param1);
        break;
    }
}


void mr_class_chassis::cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& control_msg)
{
    float left_speed = control_msg->linear.x - control_msg->angular.z * wheel_distance / 2;
    float right_speed = control_msg->linear.x + control_msg->angular.z * wheel_distance / 2;
    set_control_speed(left_speed, right_speed);
}

void mr_class_chassis::set_cutter_status(unsigned char value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.putter_state = value;
}

void mr_class_chassis::set_putter_status(unsigned char value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.cutter_state =value;
}

void mr_class_chassis::set_brake_status(unsigned char value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.brake_state = value;
}

void mr_class_chassis::init(bool is_simu, double wheel_distance, double wheel_radius)
{
    init_control_value();
    init_status_value();

    this->is_simu = is_simu;
    this->wheel_distance = wheel_distance;
    this->wheel_radius = wheel_radius;

    std::string mower_chassis_control_topic;
    this->nh_private.getParam("mower_chassis_control_topic", mower_chassis_control_topic);
    sub_chassisControl = handle.subscribe<mower_msgs::MowerChassisControl>(mower_chassis_control_topic, 10, &mr_class_chassis::cb_control, this);
    sub_cmd_vel = handle.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &mr_class_chassis::cb_cmd_vel, this);
    if (is_simu) {
        std::string sonar_front_left_topic;
        std::string sonar_front_center_topic;
        std::string sonar_front_right_topic;
        std::string sonar_rear_left_topic;
        std::string sonar_rear_center_topic;
        std::string sonar_rear_right_topic;
        std::string joint_states_topic;  

        this->nh_private.getParam("sonar_front_left_topic", sonar_front_left_topic);
        this->nh_private.getParam("sonar_front_center_topic", sonar_front_center_topic);
        this->nh_private.getParam("sonar_front_right_topic", sonar_front_right_topic);
        this->nh_private.getParam("sonar_rear_left_topic", sonar_rear_left_topic);
        this->nh_private.getParam("sonar_rear_center_topic", sonar_rear_center_topic);
        this->nh_private.getParam("sonar_rear_right_topic", sonar_rear_right_topic);
        this->nh_private.getParam("joint_states_topic", joint_states_topic);

        

        sub_sonar_front_left = handle.subscribe<sensor_msgs::Range>(sonar_front_left_topic, 10, &mr_class_chassis::cb_sonar_front_left, this);
        sub_sonar_front_center = handle.subscribe<sensor_msgs::Range>(sonar_front_center_topic, 10, &mr_class_chassis::cb_sonar_front_center, this);
        sub_sonar_front_right = handle.subscribe<sensor_msgs::Range>(sonar_front_right_topic, 10, &mr_class_chassis::cb_sonar_front_right, this);

        sub_sonar_rear_left = handle.subscribe<sensor_msgs::Range>(sonar_rear_left_topic, 10, &mr_class_chassis::cb_sonar_rear_left, this);
        sub_sonar_rear_center = handle.subscribe<sensor_msgs::Range>(sonar_rear_center_topic, 10, &mr_class_chassis::cb_sonar_rear_center, this);
        sub_sonar_rear_right = handle.subscribe<sensor_msgs::Range>(sonar_rear_right_topic, 10, &mr_class_chassis::cb_sonar_rear_right, this);

        sub_joint_states = handle.subscribe<sensor_msgs::JointState>(joint_states_topic, 10, &mr_class_chassis::cb_joint_states, this);

        pub_cmd_vel = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    }       
    else {
        std::string devname;
        int baudrate;
        nh_private.getParam("devname", devname); 
        nh_private.getParam("baudrate", baudrate); 
        startThreadRead(devname.c_str(), baudrate);
        startThreadWrite();
    } 
    
}

void mr_class_chassis::unInit()
{
    if (!is_simu) {
        stopThreadRead();
        stopThreadWrite();
    }
    
}


void mr_class_chassis::init_control_value()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
    m_value_control.left_speed = 0.0f;
    m_value_control.right_speed = 0.0f;
    m_value_control.brake_set = CHASSIS_CONTROL_BRAKE_OFF;
    m_value_control.putter_set = CHASSIS_CONTROL_PUTTER_UP;
    m_value_control.cutter_set = CHASSIS_CONTROL_CUTTER_STOP;
    memset(m_value_control.system_status, 0, 8);
}


void mr_class_chassis::init_status_value()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    m_value_status.left_speed = 0.0;
    m_value_status.right_speed = 0.0;
    m_value_status.auto_mode = CHASSIS_STATE_MODE_REMOTE_CONTROL;
    m_value_status.battery_charge_state = CHASSIS_STATE_BATTERY_NO_CHARGING;
    m_value_status.battery_volt = 12;
    m_value_status.brake_state = CHASSIS_STATE_BRAKE_OFF;
    m_value_status.cutter_state = CHASSIS_STATE_CUTTER_STOPPED;
    m_value_status.putter_state = CHASSIS_STATE_PUTTER_TOP;
}

void mr_class_chassis::startThreadRead(const char* devname, int baudrate)
{
    bool ret = m_serialport.init(devname, baudrate);

    if (!ret) {
        ROS_ERROR_STREAM("open serial port error " << devname);
        return;
    }

    threadReadRunning = true;
    threadRead = std::thread(&mr_class_chassis::fun_chassis_read, this);
}

void mr_class_chassis::startThreadWrite()
{
    threadWriteRunning = true;
    threadWrite = std::thread(&mr_class_chassis::fun_chassis_write, this);
}

void mr_class_chassis::stopThreadRead()
{
    threadReadRunning = false;
    if (threadRead.joinable()) {
        threadRead.join();
    }
    m_serialport.uninit();
}


void mr_class_chassis::stopThreadWrite()
{
    threadWriteRunning = false;
    if (threadWrite.joinable()) {
        threadWrite.join();
    }
}

void mr_class_chassis::get_chassis_status(mr_chassis_status* pvalue)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    memcpy(pvalue, &m_value_status, sizeof(mr_chassis_status));
}

void mr_class_chassis::set_chassis_status(mr_chassis_status* pvalue)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_status_data);
    memcpy(&m_value_status, pvalue, sizeof(mr_chassis_status));
    
    m_value_status.sonar_front_left    = filter_front_left.addData(pvalue->sonar_front_left);
    m_value_status.sonar_front_center  = filter_front_center.addData(pvalue->sonar_front_center);
    m_value_status.sonar_front_right   = filter_front_right.addData(pvalue->sonar_front_right);

    m_value_status.sonar_rear_left     = filter_rear_left.addData(pvalue->sonar_rear_left);
    m_value_status.sonar_rear_center   = filter_rear_center.addData(pvalue->sonar_rear_center);
    m_value_status.sonar_rear_right    = filter_rear_right.addData(pvalue->sonar_rear_right);    

    m_value_status.time_last = ros::Time::now().toNSec() / 1e6;
}




void mr_class_chassis::set_control_speed(float left_speed, float right_speed)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
    m_value_control.time_last = ros::Time::now().toNSec() / 1e6;
    m_value_control.left_speed = left_speed;
    m_value_control.right_speed= right_speed;

    sem_post(&sem_control);
    
}
void mr_class_chassis::set_control_putter(int status)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
    m_value_control.putter_set = status;

    if (is_simu) {
        if (status == CHASSIS_CONTROL_PUTTER_DOWN) {
            set_putter_status(CHASSIS_STATE_PUTTER_BOTTOM);
        }
        else if (status == CHASSIS_CONTROL_PUTTER_UP) {
            set_putter_status(CHASSIS_STATE_PUTTER_TOP);
        }
        else {
            set_putter_status(CHASSIS_STATE_PUTTER_UNKNOWN);
        }
    }
    sem_post(&sem_control);
}
void mr_class_chassis::set_control_cutter(int status)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
    m_value_control.cutter_set = status;
    if (is_simu) {
        if (status == CHASSIS_CONTROL_CUTTER_ROTATE) {
            set_cutter_status(CHASSIS_STATE_CUTTER_ROTATING);
        }
        else if (status == CHASSIS_CONTROL_CUTTER_STOP) {
            set_cutter_status(CHASSIS_STATE_CUTTER_STOPPED);
        }
        else {
            set_cutter_status(CHASSIS_STATE_CUTTER_UNKNOWN);
        }        
    }
    sem_post(&sem_control);
}
void mr_class_chassis::set_control_brake(int status)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
    m_value_control.brake_set = status;
    if (is_simu) {
        set_brake_status(status);
    }
    sem_post(&sem_control);
}
void mr_class_chassis::set_control_system_st(char* sz_status)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
    memcpy(m_value_control.system_status, sz_status, 6);
    m_value_control.system_status[6] = 0;
    sem_post(&sem_control);
}

void mr_class_chassis::get_chassis_control(mr_chassis_control* pvalue)
{
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    ts.tv_nsec += 300 * 1000000;

    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
    }    
    if (sem_timedwait(&sem_control, &ts) == 0) {
        std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
        memcpy(pvalue, &m_value_control, sizeof(mr_chassis_control));
    }
    else {
        std::lock_guard<std::recursive_mutex> lock(mutex_control_data);
        m_value_control.left_speed = 0.0;
        m_value_control.right_speed = 0.0;
    }
}


int mr_class_chassis::checkCS(const char *bytes)
{
    int num = 0;
    int len = strlen(bytes);
    for (int i = 0; i < len; i++) {
        num = (num + bytes[i]) % 256;
    }
    return num % 100;
}

void mr_class_chassis::getAccuracy(float input, char* output, int accuracy, int bufsize)
{
    char accuracyBuf[bufsize];
    snprintf(accuracyBuf, bufsize, "%%0.%df", accuracy);
    snprintf(output, bufsize, accuracyBuf, input);
}

void mr_class_chassis::assemble_control_cmd(mr_chassis_control* pvalue, char* sz_cmd)
{
    std::string ser_data;
    char sz_left_speed[16];
    char sz_right_speed[16];
    getAccuracy(fabs(pvalue->left_speed), sz_left_speed);
    getAccuracy(fabs(pvalue->right_speed), sz_right_speed);

    std::string str_left_speed = std::string(sz_left_speed);
    std::string str_right_speed = std::string(sz_right_speed);

    ser_data = "$VehicleCtl,";
    if (pvalue->left_speed >= 10) {
        ser_data = ser_data + "+" + str_left_speed + ",";
    }
    else if (pvalue->left_speed >= 0) {
        ser_data = ser_data + "+0" + str_left_speed + ",";
    }
    else if (pvalue->left_speed > -10) {
        ser_data = ser_data + "-0" + str_left_speed + ",";
    }
    else {
        ser_data = ser_data + str_left_speed + ",";
    }

    if (pvalue->right_speed >= 10) {
        ser_data = ser_data + "+" +str_right_speed + ",";
    } 
    else if (pvalue->right_speed >= 0) {
        ser_data = ser_data + "+0" +str_right_speed + ",";
    }
    else if (pvalue->right_speed > -10) {
        ser_data = ser_data + "-0" + str_right_speed + ",";
    }
    else {
        ser_data = ser_data + str_right_speed + ",";
    }

    ser_data = ser_data + std::to_string(pvalue->brake_set) + ",";
    ser_data = ser_data + std::to_string(pvalue->putter_set) + ",";
    ser_data = ser_data + std::to_string(pvalue->cutter_set) + ",";
    ser_data = ser_data + pvalue->system_status + ",*";

    std::string check_str = std::to_string(checkCS(ser_data.c_str()));
    check_str = check_str.length() == 1 ? "0" + check_str : check_str;
      
    ser_data = ser_data + check_str + ","; 
    ser_data = ser_data + "\r\n";   
    strcpy(sz_cmd, ser_data.c_str());
}


void mr_class_chassis::publish_cmd_vel(float left_speed, float right_speed)
{
    if(wheel_distance == 0) {
        return;
    }

    double linear_velocity = (left_speed + right_speed) / 2;
    double angular_velocity = (right_speed - left_speed) / wheel_distance;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    pub_cmd_vel.publish(cmd_vel);
}

