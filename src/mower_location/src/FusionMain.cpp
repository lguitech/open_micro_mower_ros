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

#include "FusionMain.h"
#include "mr_coord.h"
#include "mr_util.h"
#include "mower_msgs/MowerGnss.h"
#include <nav_msgs/Odometry.h>
#include <cmath>


#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>


static const double DIS_THRESHOLD_FLOAT = 0.1;
static const double DIS_THRESHOLD_FIXED = 0.1;

static const double THETA_THRESHOLD = M_PI / 24;
static const int INIT_COUNT = 10;

static const double PROCESS_NOISE1 = 0.01;
static const double PROCESS_NOISE3 = 0.01;

static const double MEASURE_NOSIE1 = 0.6;
static const double MEASURE_NOSIE3 = 0.3;

static const double DELTA_T = 0.1;



std::string FusionMain::getLoggerFileName(std::string strUserName) {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&now_c);

    std::stringstream ss;
    ss << "/var/tmp/logger-s.txt" ;

    return ss.str();
}

std::string FusionMain::getFormatTime() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm now_tm = *std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%H%M%S") << '.' << std::setw(3) << std::setfill('0') << now_ms.count();
    
    return oss.str();
}

void FusionMain::writeLoggerLine(
    int state, int loc_index, int sat_num, 
    double lon, double lat, double utmx, double utmy,
    double speed, double speed_wheel_left, double speed_wheel_right, double imu_theta,
    double lon_filtered, double lat_filtered, double utmx_filtered, double utmy_filtered, double theta_filtered
)
{
    std::string strFormatTime = getFormatTime();

    char buffer[256];
    sprintf(buffer, "%s, %d, %d, %02d, %.8f, %.8f, %.4f, %.4f, %.2f, %.2f, %.2f, %.1f, %.8f, %.8f, %.4f, %.4f, %.1f\r\n", 
        strFormatTime.c_str(), state, loc_index, sat_num,
        lon, lat, utmx, utmy, speed, 
        speed_wheel_left, speed_wheel_right, imu_theta * 180 / M_PI,
        lon_filtered,lat_filtered, utmx_filtered, utmy_filtered,
        theta_filtered * 180 / M_PI
    );
    fwrite(buffer, strlen(buffer), 1, fpLogger);
    fflush(fpLogger);
}

FusionMain::FusionMain()
{
    init_finished = false;
    init_count = 0;
    lastx = lasty = lastThetaGnss = lastThetaGyro = lastThetaGyroStep = 0.0;

    time_t now;
    struct tm *local_time;
    char formatted_time[20]; 

    time(&now);

    local_time = localtime(&now);

    strftime(formatted_time, sizeof(formatted_time), "%y%m%d-%H%M%S", local_time);

    char strFileSpec[128];
    sprintf(strFileSpec, "/var/tmp/logger-loc-%s.csv", formatted_time);
                                          
    fpLogger = fopen(strFileSpec, "wb");

 
    const char* head = "time, state, loc_index, satnum, lon, lat, utmx, utmy, speed, speed_wheel_left, speed_wheel_right, imu_theta, lon_filtered, lat_filtered, utmx_filtered, utmy_filtered, theta_filtered\r\n";
    fwrite(head, strlen(head), 1, fpLogger);
    fflush(fpLogger);

}

FusionMain::~FusionMain()
{

}

void FusionMain::init_filter1(double psi)
{
    filter1.initialize(psi, MEASURE_NOSIE1, PROCESS_NOISE1, DELTA_T);
}

void FusionMain::init_filter3(double utmx, double utmy)
{
    filter3.initialize(utmx, utmy, MEASURE_NOSIE3, PROCESS_NOISE3, DELTA_T);
}

double FusionMain::toNormalRegion(double angle)
{
    while(angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}


double FusionMain::normalizeDiff(double diff)
{
    while (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    while (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    return diff;
}


double FusionMain::initialize(int loc_index, double utmx, double utmy, double imu_theta, double wheel_speed)
{
    
    double result = 0.0;

    if (init_count == 0) {
        lastx = utmx;
        lasty = utmy;
        lastThetaGyro = imu_theta;
        init_count = 1;
        return result;
    }

    if (wheel_speed < 0) { 
        lastx = utmx;
        lasty = utmy;
        double diffGyro = normalizeDiff(imu_theta - lastThetaGyroStep);
        result = last_status.yaw + diffGyro;
        return result;
    }

    double dx = utmx - lastx; 
    double dy = utmy - lasty;
    double dis = std::hypot(dx, dy);
    
    if ((loc_index == LOCATION_INDEX_FIXED && dis > DIS_THRESHOLD_FIXED) ||
        (loc_index == LOCATION_INDEX_FLOAT && dis > DIS_THRESHOLD_FLOAT)
    )
    {
        double thetaGnss = std::atan2(dy, dx);
        if (thetaGnss < 0) {
            thetaGnss += 2 * M_PI;
        }

        if (init_count == 1) {
            init_filter1(thetaGnss);
        }
        else {
            double diffGyro = normalizeDiff(imu_theta - lastThetaGyro);
            result = doFilter1(diffGyro, thetaGnss);
        }
        if (init_count++ == INIT_COUNT) {
            init_finished = true;
            init_filter3(utmx, utmy);
        }
        lastx = utmx;
        lasty = utmy;
        lastThetaGyro = imu_theta;
        lastThetaGnss = thetaGnss;
    }
    else {
        double diffGyro = normalizeDiff(imu_theta - lastThetaGyroStep);
        result = last_status.yaw + diffGyro;
    }

    return result;
}


bool FusionMain::addGNSSData(EraRawGnss& param, mower_msgs::MowerGnss& msg_gnss)
{
    std::lock_guard<std::recursive_mutex> lock(mutexControl);
    static bool isFirstGNSSData = true;
    last_status.loc_index = param.loc_index;
    last_status.sat_num = param.rear_sat;
    last_status.height = param.height;
    last_status.speed = param.speed;
    last_status.roll = m_raw_imu.roll;
    last_status.pitch = m_raw_imu.pitch;
    if (isFirstGNSSData) {
        last_status.longitude = param.lon;
        last_status.latitude = param.lat;
        isFirstGNSSData = false;

        return false;
    }

    if (param.loc_index != LOCATION_INDEX_FIXED && param.loc_index != LOCATION_INDEX_FLOAT) {
        last_status.longitude = param.lon;
        last_status.latitude = param.lat;
        
        make_mower_gnss_msg(last_status, init_finished, msg_gnss);
        lastThetaGyroStep = m_raw_imu.yaw;
        return true;
    }

    
    if (fabs(m_raw_wheel_speed_left) < 0.001 && fabs(m_raw_wheel_speed_right) < 0.001) {
        make_mower_gnss_msg(last_status, init_finished, msg_gnss);
        lastThetaGyroStep = m_raw_imu.yaw;
        return true;
    }

    int logTag;
 

    double utmx = 0.0, utmy = 0.0;
    MR_Coord::getInstance()->convert_to_utm(param.lon, param.lat, &utmx, &utmy);

    double lon_filtered = param.lon, lat_filtered = param.lat;
    double utmx_filtered = utmx, utmy_filtered = utmy;

    
    if (!init_finished) {
        logTag = 0;
        last_status.yaw = initialize(param.loc_index, utmx, utmy, m_raw_imu.yaw, m_raw_wheel_speed_avg);
        last_status.longitude = param.lon;
        last_status.latitude = param.lat;
    }
    else {
        if(m_raw_wheel_speed_avg < 0) {
            doFilter3(utmx, utmy, fabs(m_raw_wheel_speed_avg), M_PI + last_status.yaw, utmx_filtered, utmy_filtered);    
            lastx = utmx_filtered;
            lasty = utmy_filtered;
        }
        else {
            doFilter3(utmx, utmy, m_raw_wheel_speed_avg, last_status.yaw, utmx_filtered, utmy_filtered);
        }
        
        MR_Coord::getInstance()->convert_to_wgs84(utmx_filtered, utmy_filtered, &lon_filtered, &lat_filtered);
        last_status.longitude = lon_filtered;
        last_status.latitude = lat_filtered;

        double dx = utmx_filtered - lastx; 
        double dy = utmy_filtered - lasty;

        double dis = std::hypot(dx, dy);
        if(
            (param.loc_index == LOCATION_INDEX_FIXED && dis > DIS_THRESHOLD_FIXED) ||
            (param.loc_index == LOCATION_INDEX_FLOAT && dis > DIS_THRESHOLD_FLOAT)
        )
        {   
            double thetaGnss = std::atan2(dy, dx);

            double diffThetaGnss = normalizeDiff(thetaGnss - lastThetaGnss);
            double diffGyro = normalizeDiff(m_raw_imu.yaw - lastThetaGyro);

            if (fabs(diffThetaGnss) < THETA_THRESHOLD && fabs(diffGyro) < THETA_THRESHOLD) {
                logTag = 3;
                last_status.yaw = doFilter1(diffGyro, thetaGnss);
            }
            else {
                logTag = 2;
                double diffGyro = normalizeDiff(m_raw_imu.yaw - lastThetaGyroStep);
                last_status.yaw = doFilter1(diffGyro, last_status.yaw + diffGyro) ;
            }
            lastx = utmx_filtered;
            lasty = utmy_filtered;

            lastThetaGnss = thetaGnss;
            lastThetaGyro = m_raw_imu.yaw;
        }
        else {
            logTag = 1;
            double diffGyro = normalizeDiff(m_raw_imu.yaw - lastThetaGyroStep);
            last_status.yaw = doFilter1(diffGyro, last_status.yaw + diffGyro) ;
        }
    }

    make_mower_gnss_msg(last_status, init_finished, msg_gnss);

    writeLoggerLine(logTag, last_status.loc_index, last_status.sat_num,
                    param.lon, param.lat, utmx, utmy, param.speed, 
                    m_raw_wheel_speed_left, m_raw_wheel_speed_right, m_raw_imu.yaw,
                    lon_filtered, lat_filtered, utmx_filtered, utmy_filtered, last_status.yaw);


    lastThetaGyroStep = m_raw_imu.yaw;

    return true;
}

double FusionMain::doFilter1(double diffGyro, double theta)
{
    static bool isFirstTheta = true;
    static double lastTheta = 0;
    if (isFirstTheta) {
        isFirstTheta = false;
        lastTheta = theta;
        return theta;
    }

    double lastTheta_normal = toNormalRegion(lastTheta);
    double diffTheta = normalizeDiff(theta - lastTheta_normal);


    lastTheta += diffTheta;

    filter1.predict(diffGyro);
    filter1.update(lastTheta);

    
    return toNormalRegion(filter1.getPost());  
}

void FusionMain::doFilter3(double utmx, double utmy, double speed, double psi, 
    double& utmx_filtered, double& utmy_filtered)
{
    filter3.predict(speed, psi);
    filter3.update(utmx, utmy);
    filter3.getPost(utmx_filtered, utmy_filtered);
}


void FusionMain::addIMUData(EraRawIMU& param)
{
    std::lock_guard<std::recursive_mutex> lock(mutexControl);

    memcpy(&m_raw_imu, &param, sizeof(EraRawIMU));

    m_raw_imu.roll = m_raw_imu.roll * M_PI / 180.0;
    m_raw_imu.pitch = m_raw_imu.pitch * M_PI / 180.0;
    m_raw_imu.yaw = m_raw_imu.yaw * M_PI / 180.0;
    
}
void FusionMain::addWheelSpeed(double speed_left, double speed_right)
{
    std::lock_guard<std::recursive_mutex> lock(mutexControl);
    m_raw_wheel_speed_left = speed_left;
    m_raw_wheel_speed_right = speed_right;
    m_raw_wheel_speed_avg = (speed_left + speed_right) / 2.0;
}


void FusionMain::make_mower_gnss_msg(mr_gnss_status& gnss_status, bool heading_inited, mower_msgs::MowerGnss& msg_gnss)
{
    msg_gnss.loc_index = gnss_status.loc_index;
    msg_gnss.sat_num = gnss_status.sat_num;
    msg_gnss.longitude = gnss_status.longitude;
    msg_gnss.latitude = gnss_status.latitude;
    msg_gnss.height = gnss_status.height;
    msg_gnss.speed = gnss_status.speed;
    msg_gnss.pitch = gnss_status.pitch;
    msg_gnss.roll = gnss_status.roll;
    msg_gnss.yaw = gnss_status.yaw;

    msg_gnss.heading_inited = heading_inited;

}
