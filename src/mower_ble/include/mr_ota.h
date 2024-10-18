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

#ifndef __MR_OTA_H__
#define __MR_OTA_H__

#include <ros/ros.h>
#include <pthread.h>
#include "mr_navi_types.h"
#include <iostream>
#include <fstream>
#include <string>
#include <curl/curl.h>
#include <boost/filesystem.hpp>

#define OTA_RESULT_OK 0
#define OTA_RESULT_NETWORK_FAIL -1
#define OTA_RESULT_DOWNLOAD_FAIL -2
#define OTA_RESULT_UNZIP_FAIL -3
#define OTA_RESULT_INPROGRESS -4

class mr_class_ota {
private:
    std::thread threadMain;
    static int threadFunction(mr_class_ota* pInstance);
public:
    bool otaRunning;
    mr_class_ota();
    ~mr_class_ota();
    int do_upgrade(const char* ssid, const char* password);
};

#endif