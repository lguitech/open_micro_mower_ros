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

#ifndef XINTUINS_FUSIONMAIN_H
#define XINTUINS_FUSIONMAIN_H

#include <ros/ros.h>
#include <mutex>
#include "mr_navi_types.h"
#include "era_types.h"
#include "kalman_filter1.h"
#include "kalman_filter3.h"
#include "mower_msgs/MowerGnss.h"


class FusionMain {
private:

    EraRawGnss m_raw_gnss;
    EraRawIMU m_raw_imu;
    double m_raw_wheel_speed_left, m_raw_wheel_speed_right, m_raw_wheel_speed_avg;

    KalmanFilter1 filter1;
    KalmanFilter3 filter3;

    bool init_finished;
    int init_count;

    double lastx, lasty;
    double lastThetaGnss, lastThetaGyro, lastThetaGyroStep;
    mr_gnss_status last_status;
    
    void make_mower_gnss_msg(mr_gnss_status& gnss_status, bool heading_inited, mower_msgs::MowerGnss& msg_gnss);

    FILE* fpLogger = nullptr;


    std::recursive_mutex mutexControl;
    bool need_reset = false;

private:
    FusionMain();    
    
    double toNormalRegion(double angle);
    double normalizeDiff(double diff);

    void init_filter1(double psi);
    void init_filter3(double utmx, double utmy);
    double initialize(int loc_index, double utmx, double utmy, double imu_theta, double wheel_speed);
    
    std::string getLoggerFileName(std::string strUserName);
    std::string getFormatTime();
    void writeLoggerLine(
        int state, int loc_index, int sat_num, 
        double lon, double lat, double utmx, double utmy,
        double speed, double speed_wheel_left, double speed_wheel_right, double imu_theta,
        double lon_filtered, double lat_filtered, double utmx_filtered, double utmy_filtered, double theta_filtered
    );
    
    double doFilter1(double diffGyro, double theta);
    void doFilter3(double utmx, double utmy, double speed, double psi, 
        double& utmx_filtered, double& utmy_filtered);

public:
    static FusionMain* getInstance() {
        static FusionMain instance;
        return &instance;
    }

    FusionMain(const FusionMain&) = delete;
    FusionMain& operator=(const FusionMain&) = delete;

    ~FusionMain();
    bool addGNSSData(EraRawGnss& param, mower_msgs::MowerGnss& msg_gnss);
    void addIMUData(EraRawIMU& param);
    void addWheelSpeed(double speed_left, double speed_right);

};


#endif

