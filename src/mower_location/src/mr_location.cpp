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
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "mr_location.h"
#include "mower_msgs/MowerGnss.h"
#include "FusionMain.h"
#include "mr_coord.h"


int mr_class_location::recv_serialport_data(unsigned char* buffer, int size)
{
    return m_serialport.recv(buffer, size);
}

int mr_class_location::send_serialport_data(unsigned char* buffer, int size)
{
    return m_serialport.send(buffer, size);
}


void mr_class_location::init(bool is_simu)
{
    std::string mower_chassis_state_topic;
    std::string mower_map_meta_topic;

    std::string mower_gnss_topic;
    std::string mower_odom_topic;
	std::string mower_aruco_detect_topic;
	std::string mower_home_update_topic;

    this->is_simu = is_simu;

    nh_private.getParam("mower_gnss_topic", mower_gnss_topic);
    nh_private.getParam("mower_odom_topic", mower_odom_topic);
    nh_private.getParam("mower_chassis_state_topic", mower_chassis_state_topic);
    nh_private.getParam("mower_map_meta_topic", mower_map_meta_topic);
	nh_private.getParam("mower_aruco_detect_topic", mower_aruco_detect_topic);
	nh_private.getParam("mower_home_update_topic", mower_home_update_topic);


    pub_mower_gnss = handle.advertise<mower_msgs::MowerGnss>(mower_gnss_topic, 10);
    pub_mower_odom = handle.advertise<nav_msgs::Odometry>(mower_odom_topic, 10);
	pub_update_home = handle.advertise<geometry_msgs::Pose>(mower_home_update_topic, 10);

    sub_chassis_status = handle.subscribe<mower_msgs::MowerChassisState>(mower_chassis_state_topic, 10, &mr_class_location::cb_chassis_state, this);
    sub_mower_map_meta = handle.subscribe<mower_msgs::MowerMapMeta>(mower_map_meta_topic, 10, &mr_class_location::cb_map_meta, this);    

	sub_aruco_detect = handle.subscribe<geometry_msgs::Pose>(mower_aruco_detect_topic, 10, &mr_class_location::cb_aruco_detect, this);

    if (is_simu) {
        std::string gnss_topic;
        std::string imu_topic;

        nh_private.getParam("gnss_topic", gnss_topic);
        nh_private.getParam("imu_topic", imu_topic);

        sub_gnss = handle.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 10, &mr_class_location::cb_gnss, this);
        sub_imu = handle.subscribe<sensor_msgs::Imu>(imu_topic, 10, &mr_class_location::cb_imu, this);
    }
    else {
        std::string devname;
        int baudrate;

        nh_private.getParam("devname", devname); 
        nh_private.getParam("baudrate", baudrate); 
        start(devname.c_str(), baudrate);
    }
}

void mr_class_location::unInit()
{
    if (!is_simu) {
        stop();
    }
}


#include <deque>
std::deque<double> queueSpeed;
const int SPEED_QUEUE_MAX = 5;
double lastx = 0;
double lasty = 0;
bool isFirst = true;
const double DELTA_T = 0.1;
double sumSpeed = 0;
double calc_speed(double lon, double lat)
{
    double utmx, utmy;
    MR_Coord::getInstance()->convert_to_utm(lon, lat, &utmx, &utmy);
    if (isFirst) {
        lastx = utmx;
        lasty = utmy;
        isFirst = false;
        return 0;
    }
    double dis = std::hypot(utmx - lastx, utmy - lasty);
    lastx = utmx;
    lasty = utmy;
    double speed = dis / DELTA_T;
    
    queueSpeed.push_back(speed);
    sumSpeed += speed; 

    if (queueSpeed.size() == SPEED_QUEUE_MAX) {
        double head = queueSpeed.front();
        queueSpeed.pop_front();
        sumSpeed -= head;
    }
    double avgSpeed = sumSpeed / queueSpeed.size();
    return avgSpeed;
}

void mr_class_location::cb_gnss(const sensor_msgs::NavSatFix::ConstPtr& navStaFix)
{
    EraRawGnss rawGnss;
    rawGnss.lon = navStaFix->longitude;
    rawGnss.lat = navStaFix->latitude;
    rawGnss.height = navStaFix->altitude;

    rawGnss.speed = calc_speed(rawGnss.lon, rawGnss.lat);

    rawGnss.loc_index = LOCATION_INDEX_FIXED;
    rawGnss.rear_sat = 48;

    mower_msgs::MowerGnss msg_gnss;
    bool ret = FusionMain::getInstance()->addGNSSData(rawGnss, msg_gnss);
    if (ret) {
        publish_location(msg_gnss);
    }
}

void mr_class_location::cb_imu(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    tf::Quaternion RQ2;  
    double roll,pitch,yaw;
    tf::Quaternion imu_quaternion(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
    tf::Matrix3x3(imu_quaternion).getRPY(roll, pitch, yaw);
    EraRawIMU rawIMU;
    rawIMU.ax = imu_msg->linear_acceleration.x;
    rawIMU.ay = imu_msg->linear_acceleration.y;
    rawIMU.az = imu_msg->linear_acceleration.z;
    rawIMU.gx = imu_msg->angular_velocity.x;
    rawIMU.gy = imu_msg->angular_velocity.y;
    rawIMU.gz = imu_msg->angular_velocity.z;

    yaw -= 4.0;

    rawIMU.pitch = pitch * 180.0 / M_PI;
    rawIMU.roll = roll * 180.0 / M_PI;
    rawIMU.yaw = yaw * 180.0 / M_PI;

    

    if (rawIMU.yaw < 0) {
        rawIMU.yaw += 360.0;
    }
    FusionMain::getInstance()->addIMUData(rawIMU);
}

void mr_class_location::cb_chassis_state(const mower_msgs::MowerChassisState::ConstPtr& msg)
{
    FusionMain::getInstance()->addWheelSpeed(msg->left_speed, msg->right_speed);
}

void mr_class_location::cb_map_meta(const mower_msgs::MowerMapMeta::ConstPtr& msg)
{
    pt_original.x = msg->original.x;
    pt_original.y = msg->original.y;
    map_initialized = true;
}

void mr_class_location::fun_read_location() 
{   
    while(threadReadRunning) {
        unsigned char buffer[2048];
        int len = recv_serialport_data(buffer, 2048);
        if (len > 0) {
            parse_data(buffer, len);
        }
        else {
            usleep(50*1000);
        }
    }
}


void mr_class_location::start(const char* devname, int baudrate)
{
    bool ret = m_serialport.init(devname, baudrate);
    if (!ret) {
        ROS_ERROR_STREAM("open serial port error " << devname);
        return;
    }

    threadReadRunning = true;
    threadRead = std::thread(&mr_class_location::fun_read_location, this);
}

void mr_class_location::stop()
{
    threadReadRunning = false;
    if (threadRead.joinable()) {
        threadRead.join();
    }
    m_serialport.uninit();
}


unsigned char mr_class_location::calc_checksum(const unsigned char* sentence, int len)
{
    unsigned char checksum = 0;

    for (int i = 0; i < len; i++)
    {
        checksum ^= sentence[i];
    }
    return checksum;
}

void mr_class_location::do_parse()
{

    unsigned char cs = calc_checksum(parse_buffer+4, 44);
    if (parse_buffer[3] != cs) {
        ROS_INFO_STREAM("check sum error " << parse_buffer[2]);
        return;
    }

    if (parse_buffer[2] == 1) {
        int offset = 4;
        raw_gnss.timestamp  = *(unsigned int*)(parse_buffer + offset); offset += 4;
        raw_gnss.loc_index  = *(parse_buffer + offset++);
        raw_gnss.heading_index = *(parse_buffer + offset++);
        raw_gnss.front_sat  = *(parse_buffer + offset++);
        raw_gnss.rear_sat   = *(parse_buffer + offset++);

        raw_gnss.lon        = *(double*)(parse_buffer + offset); offset += 8;
        raw_gnss.lat        = *(double*)(parse_buffer + offset); offset += 8;
        raw_gnss.speed      = *(float*)(parse_buffer + offset); offset += 4;
        raw_gnss.height     = *(float*)(parse_buffer + offset); offset += 4;
        raw_gnss.heading    = *(float*)(parse_buffer + offset); offset += 4;
        raw_gnss.pitch      = *(float*)(parse_buffer + offset); offset += 4;

        mower_msgs::MowerGnss msg_gnss;
        bool ret = FusionMain::getInstance()->addGNSSData(raw_gnss, msg_gnss);
        if (ret) {
            publish_location(msg_gnss);
        }
    }
    else if (parse_buffer[2] == 3) {
        int offset = 4;
        raw_imu.timestamp = *(unsigned int*)(parse_buffer + offset); offset += 4;
        raw_imu.pitch = *(float*)(parse_buffer + offset); offset += 4;
        raw_imu.roll = *(float*)(parse_buffer + offset); offset += 4;
        raw_imu.yaw = *(float*)(parse_buffer + offset); offset += 4;

        raw_imu.ax = *(float*)(parse_buffer + offset); offset += 4;
        raw_imu.ay = *(float*)(parse_buffer + offset); offset += 4;
        raw_imu.az = *(float*)(parse_buffer + offset); offset += 4;

        raw_imu.gx = *(float*)(parse_buffer + offset); offset += 4;
        raw_imu.gy = *(float*)(parse_buffer + offset); offset += 4;
        raw_imu.gz = *(float*)(parse_buffer + offset); offset += 4;

        raw_imu.temp = *(float*)(parse_buffer + offset);

        raw_imu.yaw = 90.0 - raw_imu.yaw;
        if (raw_imu.yaw < 0) {
            raw_imu.yaw += 360.0;
        }
        FusionMain::getInstance()->addIMUData(raw_imu);

    }
}


int mr_class_location::find_begin_tag(const unsigned char* pData, int len)
{
    int index_begin = 0;
    for (; index_begin < len; index_begin ++) {
        if (pData[index_begin] == 0xAA) {
            break;
        }
    }
    if (index_begin < len) {
        return index_begin;
    }
    else {
        return -1;
    }
}



void mr_class_location::parse_data(unsigned char* pData, int len)
{
    int pos_offset = 0;
    while(pos_offset < len) {
        if (pos_tail == 0) {
            int index_begin = find_begin_tag(pData + pos_offset, len - pos_offset);
            if (index_begin == -1) {
                ROS_INFO_STREAM("head not found!");
                return; //head not found
            }
            pos_offset += index_begin;
        }

        int copy_num = std::min(PARSE_BUFFER_LEN - pos_tail, len - pos_offset);
        memcpy(parse_buffer + pos_tail, pData + pos_offset, copy_num);
        pos_tail += copy_num;
        if (pos_tail == PARSE_BUFFER_LEN) {
            do_parse();
            pos_tail = 0;
        }
        pos_offset += copy_num;
    }
}



void mr_class_location::convertGnss2OdomCoord(mr_point& gnssPoint, mr_point& odomPoint)
{
    //转换为utm
    double pos_utmx;
    double pos_utmy;
    MR_Coord::getInstance()->convert_to_utm(gnssPoint.x, gnssPoint.y, &pos_utmx, &pos_utmy);

    odomPoint.x = pos_utmx - pt_original.x;
    odomPoint.y = pos_utmy - pt_original.y;
}

void mr_class_location::do_filter(mower_msgs::MowerGnss& msg, Eigen::Vector3d& vecPose)
{
    mr_point gnssPoint(msg.longitude, msg.latitude);
    mr_point odomPoint;

    convertGnss2OdomCoord(gnssPoint, odomPoint);

    vecPose << odomPoint.x, odomPoint.y, msg.yaw;
}

void mr_class_location::build_publish_odom(Eigen::Vector3d& vecPose)
{
    nav_msgs::Odometry odomMsg;

    odomMsg.header.stamp = ros::Time::now();
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_footprint";
    odomMsg.pose.pose.position.x = vecPose(0);
    odomMsg.pose.pose.position.y = vecPose(1);
    odomMsg.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion quat = 
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, vecPose(2));
    odomMsg.pose.pose.orientation = quat;

    pub_mower_odom.publish(odomMsg);

    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(vecPose(0), vecPose(1), 0.0));
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, vecPose(2));

    transform.setRotation(quaternion);

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}



void mr_class_location::publish_location(mower_msgs::MowerGnss& msg_gnss)
{
    pub_mower_gnss.publish(msg_gnss);
    if (map_initialized) {
        Eigen::Vector3d vecPose;
        do_filter(msg_gnss, vecPose);
        build_publish_odom(vecPose);
    }

}







void mr_class_location::cb_aruco_detect(const geometry_msgs::Pose::ConstPtr& msg)
{
	double yaw, pitch, roll;

	tf2::getEulerYPR(msg->orientation, yaw, pitch, roll);

	double marker_x = msg->position.z;
	double marker_y = -msg->position.x;
	double marker_z = -msg->position.y;
	
	double marker_roll = 0;
	double marker_pitch = 0;
	double marker_yaw = -pitch;

	tf::StampedTransform transform;
	try {
        tf_listener.waitForTransform("odom", "cam_link", ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform("odom", "cam_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Transform exception: %s", ex.what());
        return;
    }

	geometry_msgs::Pose pose_in_cam_coord, pose_in_odom_coord;
	tf::Pose tf_input_pose, tf_output_pose;

	pose_in_cam_coord.position.x = marker_x;
	pose_in_cam_coord.position.y = marker_y;
	pose_in_cam_coord.position.z = marker_z;

	tf::Quaternion quaternion;
	quaternion.setRPY(marker_roll, marker_pitch,marker_yaw);
	tf::quaternionTFToMsg(quaternion, pose_in_cam_coord.orientation);	

    tf::poseMsgToTF(pose_in_cam_coord, tf_input_pose);

    tf_output_pose = transform * tf_input_pose;

    tf::poseTFToMsg(tf_output_pose, pose_in_odom_coord);
	
	tf2::getEulerYPR(pose_in_odom_coord.orientation, yaw, pitch, roll);
	pose_in_odom_coord.position.x -= POS_DIFF_DOCK * cos (yaw);
	pose_in_odom_coord.position.y -= POS_DIFF_DOCK * sin (yaw);


	pub_update_home.publish(pose_in_odom_coord);
}




