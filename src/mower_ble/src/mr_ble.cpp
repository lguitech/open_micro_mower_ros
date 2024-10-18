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
#include <fstream>
#include <assert.h>
#include "mower_msgs/MowerChassisControl.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "cjsonobject.h"
#include "mr_ble.h"
#include "mr_bletool.h"
#include "mr_util.h"
#include "minilzo.h"
#include "mr_ota.h"


int mr_class_ble::recv_serialport_data(unsigned char* buffer, int size)
{
    return m_serialport.recv(buffer, size);
}

int mr_class_ble::send_serialport_data(unsigned char* buffer, int size)
{
    return m_serialport.send(buffer, size);
}


void mr_class_ble::cb_gnss(const mower_msgs::MowerGnss::ConstPtr& msg)
{
    mr_gnss_status gnss_status;
    gnss_status.loc_index = msg->loc_index;
    gnss_status.sat_num = msg->sat_num;

    gnss_status.longitude = msg->longitude;
    gnss_status.latitude = msg->latitude;
    gnss_status.height = msg->height;
    
    gnss_status.pitch = msg->pitch;
    gnss_status.roll = msg->roll;
    gnss_status.yaw = msg->yaw;
    
    set_gnss_value(gnss_status);
}

void mr_class_ble::cb_chassis_state(const mower_msgs::MowerChassisState::ConstPtr& msg)
{
    mr_chassis_status chassis_status;
    
    chassis_status.left_speed = msg->left_speed;
    chassis_status.right_speed = msg->right_speed;

    chassis_status.putter_state = msg->putter_state;
    chassis_status.cutter_state = msg->cutter_state;
    chassis_status.auto_mode = msg->auto_mode;
    chassis_status.brake_state = msg->brake_state;

    chassis_status.safe_edge_back = msg->safe_edge_back;
    chassis_status.safe_edge_front = msg->safe_edge_front;
    chassis_status.safe_edge_left = msg->safe_edge_left;
    chassis_status.safe_edge_right = msg->safe_edge_right;

    chassis_status.soc = msg->soc;
    chassis_status.battery_charge_state = msg->battery_charge_state;
    chassis_status.battery_volt = msg->battery_volt;
    
    strcpy(chassis_status.fault_code, msg->fault_code.c_str());

    chassis_status.sonar_front_left = msg->sonar_front_left;
    chassis_status.sonar_front_center = msg->sonar_front_center;
    chassis_status.sonar_front_right = msg->sonar_front_right;

    chassis_status.sonar_rear_left = msg->sonar_rear_left;
    chassis_status.sonar_rear_center = msg->sonar_rear_center;
    chassis_status.sonar_rear_right = msg->sonar_rear_right;


    set_chassis_value(chassis_status);
}

void mr_class_ble::cb_work_state(const std_msgs::Int32::ConstPtr& msg)
{
    robot_state = msg->data;
}

void mr_class_ble::readVersion() 
{
    std::string filePath = std::getenv("HOME") + std::string("/open_micro_mower_ros/release_info.json");
    std::ifstream file(filePath);
    if (!file.is_open()) {
        ROS_ERROR_STREAM( "Failed to open file: " << filePath);
        strVersion = "unknown";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string strContent = buffer.str();
    neb::CJsonObject json(strContent.c_str());

    bool result = json.Get("version", strVersion);
    if (!result) {
        strVersion = "unknown";
    }
}

void mr_class_ble::init(double max_linear_speed, double wheel_distance)
{
    this->max_linear_speed = max_linear_speed;
    this->wheel_distance = wheel_distance;

    std::string devname;
    int baudrate;

    std::string mower_gnss_topic;
    std::string mower_chassis_state_topic;
    
    std::string mower_work_state_topic;
    std::string mower_work_control_topic;
    std::string mower_map_update_topic;


    nh_private.getParam("devname", devname); 
    nh_private.getParam("baudrate", baudrate); 

    nh_private.getParam("mower_gnss_topic", mower_gnss_topic);
    nh_private.getParam("mower_chassis_state_topic", mower_chassis_state_topic);
    nh_private.getParam("mower_work_state_topic", mower_work_state_topic);

    nh_private.getParam("mower_work_control_topic", mower_work_control_topic);
    nh_private.getParam("mower_map_update_topic", mower_map_update_topic);
    
    nh_private.getParam("filespec", m_mapFilePath);

    sub_gnss = handle.subscribe(mower_gnss_topic, 10, &mr_class_ble::cb_gnss, this);
    sub_chassis_state = handle.subscribe(mower_chassis_state_topic, 10, &mr_class_ble::cb_chassis_state, this);
    sub_work_state = handle.subscribe(mower_work_state_topic, 10, &mr_class_ble::cb_work_state, this);

    pub_cmd_vel = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);

    pub_work_control = handle.advertise<std_msgs::Int32>(mower_work_control_topic, 10, true);

    pub_map_update = handle.advertise<std_msgs::String>(mower_map_update_topic, 10, true);

    readVersion();

    startMainThread(devname.c_str(), baudrate);
}

void mr_class_ble::unInit()
{
    stopMainThread();
}


void mr_class_ble::startMainThread(const char* devname, int baudrate)
{
    bool ret = m_serialport.init(devname, baudrate);
    if (!ret) {
        ROS_ERROR_STREAM("init serial port error " << devname);
        return;
    }
    
    threadRunning = true;
    threadMain = std::thread(&mr_class_ble::threadFunction, this);
}

void mr_class_ble::stopMainThread()
{
    threadRunning = false;
    if (threadMain.joinable()) {
        threadMain.join();
    }
    m_serialport.uninit();

}

void mr_class_ble::calc_index(int num_total, int points_per_pack, int& index_start, int& index_end)
{
    if (num_total == index_end + 1) {
        index_start = -1;
        index_end = -1;
        return;
    }

    if (index_start == -1) {
        index_start = 0;
    }
    else {
        index_start = index_end + 1;
    }
    index_end = std::min(num_total-1, index_start + points_per_pack - 1);
}


void mr_class_ble::threadFunction()
{
    mr_class_bletool ble_tool;
    int num_total_bytes = 0;
    unsigned char* pAttachData = NULL;

    int index_start_last = -1;
    int index_end_last = -1;
    int bytes_per_pack = 112; 

    bool is_receiving_mass = false;

    uint32_t timestamp_last1 = 0;
    uint32_t timestamp_last2 = 0;

    int waiting_counter = 0;

    while(threadRunning) {
        if (is_receiving_mass) {
            if (waiting_counter++ >= 40) {
                waiting_counter = 0;
                is_receiving_mass = false;
            }
        }          
         unsigned char buffer[2048];

        int len = recv_serialport_data(buffer, 2048);
        
        if (len > 0) {
            mr_ble_data ble_data;
            ble_tool.append_data(buffer, len);
            while(ble_tool.parse_data(ble_data)) {
                int angle = 0;
                int strength = 0;

                switch (ble_data.reqid) {
                case PHONE_REQ_STATE_RESET:
                    ROS_INFO_STREAM("ble received REQ_STATE_RESET");
                    onBLEReceivedWorkControl(ROBOT_WORK_CONTROL_RESET);
                    break;
                case PHONE_REQ_GO_HOME:
                    ROS_INFO_STREAM("ble received REQ_GO_HOME");
                    onBLEReceivedWorkControl(ROBOT_WORK_CONTROL_GOHOME);
                    break;
                case PHONE_REQ_START_WORK:
                    ROS_INFO_STREAM("ble received REQ_START_WORK");
                    onBLEReceivedWorkControl(ROBOT_WORK_CONTROL_START);
                    break;
                case PHONE_REQ_PAUSE_WORK:
                    ROS_INFO_STREAM("ble received REQ_STOP_WORK");
                    onBLEReceivedWorkControl(ROBOT_WORK_CONTROL_PAUSE);
                     break; 
                case PHONE_REQ_REMOTE_CONTROL:
                    angle = mower::byte2int((const unsigned char*)ble_data.bufAttach);
                    strength = mower::byte2int((const unsigned char*)ble_data.bufAttach + 4);
                    onBLEReceivedChassisControl(angle, strength);
                    break;
    
                case PHONE_REQ_SEND_MAP_INFO:   
                    index_start_last = -1;
                    index_end_last = -1;

                    num_total_bytes = mower::byte2int(ble_data.bufAttach);
                    
                    if (pAttachData != NULL) {
                        free(pAttachData);
                    }
                    pAttachData = (unsigned char*)malloc(num_total_bytes);
                    
                    calc_index(num_total_bytes, bytes_per_pack, index_start_last, index_end_last);
                    send_map_request(index_start_last, index_end_last);

                    is_receiving_mass = true;
                    waiting_counter = 0;
                    break;    
                case PHONE_REQ_SEND_MAP_DATA:
                    if (is_receiving_mass) {
                        if (mower::byte2int(ble_data.bufAttach) != index_start_last  || 
                            mower::byte2int(ble_data.bufAttach + 4) != index_end_last) {
                        }
                        else {
                            int len = index_end_last - index_start_last + 1;
                            memcpy(pAttachData + index_start_last, ble_data.bufAttach + 8, len);

                            calc_index(num_total_bytes, bytes_per_pack, index_start_last, index_end_last);
                            send_map_request(index_start_last, index_end_last);
                            if (index_start_last == -1 && index_end_last == -1) {
                                onReceivedData(pAttachData, num_total_bytes);
                                is_receiving_mass = false;
                            }
                            waiting_counter = 0;
                        }
                    }
                    break; 
                case PHONE_REQ_OTA:
                    {
                    char ssid[128];
                    char password[128];
                    char* strInput = (char*)ble_data.bufAttach;
                    const char* commaPos = std::strchr(strInput, ',');
                    size_t firstLength = commaPos - (char*)ble_data.bufAttach;
                    size_t secondLength = std::strlen(commaPos + 1);


                    std::strncpy(ssid, strInput, firstLength);
                    ssid[firstLength] = '\0';

                    std::strcpy(password, commaPos + 1);
                    ROS_INFO_STREAM("WIFI " << ssid << ", " << password);
                    int result = req_ota(ssid, password);
                    send_ota_result(result);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    if (result == OTA_RESULT_OK) {
                        //reboot
                        system("sudo reboot");
                    }
                    break;
                    }   
                }                 
            }
        }
        else {
            usleep(50 * 1000);
        }
        if (!is_receiving_mass) {
            uint32_t current_timestamp = ros::Time::now().toNSec() / 1000000;
            if (current_timestamp - timestamp_last1 >= 500) {
                send_gnss();
                timestamp_last1 = current_timestamp;
            }
            if (current_timestamp - timestamp_last2 >= 1000) {
                send_chassis();
                timestamp_last2 = current_timestamp;
            }
        }        
    }
}

void mr_class_ble::set_gnss_value(mr_gnss_status& value)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_gnss);
    memcpy(&m_gnss_status, &value, sizeof(mr_gnss_status));
}

void mr_class_ble::get_gnss_value(mr_gnss_status& value)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_gnss);
    memcpy(&value, &m_gnss_status, sizeof(mr_gnss_status));

}

void mr_class_ble::set_chassis_value(mr_chassis_status& value)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_chassis);
    memcpy(&m_chassis_status, &value, sizeof(mr_chassis_status));
}
void mr_class_ble::get_chassis_value(mr_chassis_status& value)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_chassis);
    memcpy(&value, &m_chassis_status, sizeof(mr_chassis_status));
}


int mr_class_ble::checkCS(const char *bytes)
{
    int num = 0;
    int len = strlen(bytes);
    for (int i = 0; i < len; i++) {
        num = (num + bytes[i]) % 256;
    }
    return num % 100;
}

void mr_class_ble::send_map_request(int index_start, int index_end)
{
    char buffer[256];
    sprintf(buffer, "$pth,%d,%d*", index_start, index_end);
    std::string check_str = std::to_string(checkCS(buffer));
    check_str = check_str.length() == 1 ? "0" + check_str : check_str;
    strcat(buffer, check_str.c_str());
    strcat(buffer, "\r\n");

    send_serialport_data((unsigned char*)buffer, strlen(buffer));
}



void mr_class_ble::send_gnss()
{
    mr_gnss_status gnss_status;
    get_gnss_value(gnss_status);
    time_t time_curr = time(NULL);
    if (time_curr - gnss_status.time_last < 3) {
        char buffer[256];
        sprintf(buffer, "$loc,%.8lf,%.8lf,%.2lf,%.1lf,%.1lf,%.1lf,%d,%d*",
            gnss_status.longitude, gnss_status.latitude, gnss_status.height, 
            gnss_status.yaw, gnss_status.pitch, gnss_status.roll,
            gnss_status.sat_num, gnss_status.loc_index
        );
        std::string check_str = std::to_string(checkCS(buffer));
        check_str = check_str.length() == 1 ? "0" + check_str : check_str;
        strcat(buffer, check_str.c_str());
        strcat(buffer, "\r\n");
        send_serialport_data((unsigned char*)buffer, strlen(buffer));
    }
}

void mr_class_ble::send_chassis()
{
    mr_chassis_status chassis_status;
    get_chassis_value(chassis_status);
    time_t time_curr = time(NULL);

	char buffer[1024];
	sprintf(buffer, "$chs,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s*",
	robot_state,
	chassis_status.left_speed, chassis_status.right_speed,
	chassis_status.putter_state, chassis_status.cutter_state,
	chassis_status.auto_mode, chassis_status.brake_state,

	chassis_status.safe_edge_front, chassis_status.safe_edge_back,
	chassis_status.safe_edge_left, chassis_status.safe_edge_right,
	chassis_status.soc, chassis_status.battery_volt,
	chassis_status.battery_charge_state,
	chassis_status.sonar_front_left,
	chassis_status.sonar_front_center,
	chassis_status.sonar_front_right,
	chassis_status.sonar_rear_right, 
	chassis_status.sonar_rear_center,
	chassis_status.sonar_rear_left,
	strVersion.c_str());

	std::string check_str = std::to_string(checkCS(buffer));
	check_str = check_str.length() == 1 ? "0" + check_str : check_str;
	strcat(buffer, check_str.c_str());
	strcat(buffer, "\r\n");

	send_serialport_data((unsigned char*)buffer, strlen(buffer));
}

void mr_class_ble::send_ota_result(int value)
{
    char buffer[1024];
    sprintf(buffer, "$ota,%d*", value);

    std::string check_str = std::to_string(checkCS(buffer));
    check_str = check_str.length() == 1 ? "0" + check_str : check_str;
    strcat(buffer, check_str.c_str());
    strcat(buffer, "\r\n");

    send_serialport_data((unsigned char*)buffer, strlen(buffer));
}

void mr_class_ble::onReceivedData(unsigned char* pAttachData, int len)
{
    unsigned long len_unzip;
    unsigned char* pUnzip = (unsigned char*)malloc(4*1024*1024);
    lzo1x_decompress((unsigned char*)pAttachData, len, (unsigned char*)pUnzip, &len_unzip, NULL);

    pUnzip[len_unzip] = 0;
    
    std::ofstream outFile(m_mapFilePath, std::ios::binary);
    if (outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(pUnzip), len_unzip);            
        outFile.close();
    }
    else {
        return;
    }

    onBLEReceivedMapUpdate();

    free(pUnzip);
}


void mr_class_ble::onBLEReceivedMapUpdate()
{
    int pack_cap = 1024;
    std_msgs::String msg;
    msg.data = m_mapFilePath;
    pub_map_update.publish(msg);
}

void mr_class_ble::onBLEReceivedWorkControl(int value)
{
    std_msgs::Int32 msg;
    msg.data = value;
    pub_work_control.publish(msg);
}


void mr_class_ble::convert_cmd_vel(float left_speed, float right_speed, geometry_msgs::Twist& cmd_vel)
{
    assert(wheel_distance != 0);
    double linear_velocity = (left_speed + right_speed) / 2;
    double angular_velocity = (right_speed - left_speed) / wheel_distance;

    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
}


void mr_class_ble::steer_convert_manual(int deg, int strength, float& left_speed, float& right_speed)
{
    int control_region = 0;
    if (deg >0 && deg<=90) {        
        control_region = 0;
    }
    else if (deg>90 && deg<=180) {  
        control_region = 1;
    }
    else if (deg>180 && deg<=190) {
        control_region = 2;
    }
    else if (deg>190 && deg<=225) { 
        control_region = 3;
    }
    else if (deg>225 && deg<=270) { 
        control_region = 4;
    }
    else if (deg>270 && deg<=315) { 
        control_region = 5;
    }
    else if (deg>315 && deg<=350) { 
        control_region = 6;
    }
    else if (deg>350 && deg<=360) { 
        control_region = 7;
    }

    float diff_forward = max_linear_speed / 90.0;
    float diff_backward = max_linear_speed / 45.0;
    switch(control_region) {
    case 0: 
        left_speed = max_linear_speed;
        right_speed = deg * diff_forward;
        break;
    case 1: 
        right_speed = max_linear_speed;
        left_speed = (180-deg) * diff_forward;
        break;
    case 2: 
        right_speed = max_linear_speed;
        left_speed = 0;
        break;
    case 3: 
        right_speed = 0;
        left_speed = 0;
        break;
    case 4:
        right_speed = -1 * max_linear_speed;
        left_speed =  -1 * (deg-225) * diff_backward;
        break;
    case 5: 
        left_speed = -1 * max_linear_speed;
        right_speed = -1 * (315-deg) * diff_backward;
        break;
    case 6: 
        left_speed = 0;
        right_speed = 0;
        break;
    case 7: 
        left_speed = max_linear_speed;
        right_speed = 0;
        break;
    }
    left_speed = left_speed * strength / 100.0;
    right_speed = right_speed * strength / 100.0;
}

void mr_class_ble::onBLEReceivedChassisControl(int angle, int strength)
{
    float left_speed, right_speed;
    steer_convert_manual(angle, strength, left_speed, right_speed);
    geometry_msgs::Twist cmd_vel;
    convert_cmd_vel(left_speed, right_speed, cmd_vel);
    pub_cmd_vel.publish(cmd_vel);
}



int mr_class_ble::req_ota(const char* ssid, const char* password)
{
    mr_class_ota ota;
    return ota.do_upgrade(ssid, password);   
}