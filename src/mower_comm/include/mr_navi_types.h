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

#ifndef __MR_NAVI_TYPES_H__
#define  __MR_NAVI_TYPES_H__

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <thread>
#include <vector>
#include <string>
#include <memory.h>
#include <time.h>
#include <math.h>
#include <semaphore.h>
#include <assert.h>
#include <unordered_set>
#include <opencv2/opencv.hpp>


#define PREDEF_RESOLUTION 0.2  
#define PREDEF_RESOLUTION_INVERSE 5.0  



#define OBSTACLE_ID_FRONT_LEFT 0
#define OBSTACLE_ID_FRONT_CENTER 1
#define OBSTACLE_ID_FRONT_RIGHT 2

#define OBSTACLE_ID_REAR_LEFT 3
#define OBSTACLE_ID_REAR_CENTER 4
#define OBSTACLE_ID_REAR_RIGHT 5

#define BEHAV_RESULT_ERROR  -1
#define BEHAV_RESULT_DEFAULT  0
#define BEHAV_RESULT_FINISHED 1


#define MAP_CELL_FREE 0
#define MAP_CELL_OCCUPIED 100
#define MAP_CELL_UNKNOWN -1


#define PHONE_REQ_START_WORK 1 
#define PHONE_REQ_PAUSE_WORK 3 
#define PHONE_REQ_REMOTE_CONTROL 4
#define PHONE_REQ_SEND_MAP_INFO 5 
#define PHONE_REQ_SEND_MAP_DATA 6 
#define PHONE_REQ_GO_HOME 9       
#define PHONE_REQ_STATE_RESET 10  
#define PHONE_REQ_OTA 11          


#define ROBOT_WORK_CONTROL_START 1
#define ROBOT_WORK_CONTROL_PAUSE 3
#define ROBOT_WORK_CONTROL_GOHOME 9
#define ROBOT_WORK_CONTROL_RESET 10


#define ROBOT_STATE_UNKNOWN -1
#define ROBOT_STATE_IDLE  0
#define ROBOT_STATE_WORKING  1
#define ROBOT_STATE_PAUSED  2
#define ROBOT_STATE_STUCK  3  
#define ROBOT_STATE_FAULT 4  
#define ROBOT_STATE_STRUGGLING 5 
#define ROBOT_STATE_LOCATING 6
#define ROBOT_STATE_TRANSFERRING 7
#define ROBOT_STATE_DOCKING 8
#define ROBOT_STATE_GOINGHOME 9


#define LOCATION_INDEX_INVALID 0
#define LOCATION_INDEX_SP 1
#define LOCATION_INDEX_FLOAT 2
#define LOCATION_INDEX_FIXED 3

#define HEADING_INDEX_INVALID 0
#define HEADING_INDEX_SP 1
#define HEADING_INDEX_FLOAT 2
#define HEADING_INDEX_FIXED 3

#define CHASSIS_STATE_PUTTER_UNKNOWN    -1
#define CHASSIS_STATE_PUTTER_BOTTOM     0
#define CHASSIS_STATE_PUTTER_TOP        1
#define CHASSIS_STATE_PUTTER_FALLING    2
#define CHASSIS_STATE_PUTTER_RISING     3

#define CHASSIS_STATE_CUTTER_UNKNOWN    -1
#define CHASSIS_STATE_CUTTER_STOPPED    0
#define CHASSIS_STATE_CUTTER_ROTATING   1

#define CHASSIS_STATE_MODE_REMOTE_CONTROL    0
#define CHASSIS_STATE_MODE_AUTO_CONTROL      1

#define CHASSIS_STATE_BRAKE_OFF  0
#define CHASSIS_STATE_BRAKE_ON     1

#define CHASSIS_STATE_SAFE_EDGE_OK        0
#define CHASSIS_STATE_SAFE_EDGE_TOUCHED   1
#define CHASSIS_STATE_SAFE_EDGE_DAMAGED   2

#define CHASSIS_STATE_BATTERY_NO_CHARGING 0
#define CHASSIS_STATE_BATTERY_CHARGING 1


#define CHASSIS_CONTROL_TYPE_PUTTER 13
#define CHASSIS_CONTROL_TYPE_CUTTER 14
#define CHASSIS_CONTROL_TYPE_BRAKE 15

#define CHASSIS_CONTROL_PUTTER_DOWN 2
#define CHASSIS_CONTROL_PUTTER_UP 1

#define CHASSIS_CONTROL_CUTTER_ROTATE 1
#define CHASSIS_CONTROL_CUTTER_STOP 0

#define CHASSIS_CONTROL_BRAKE_ON 1
#define CHASSIS_CONTROL_BRAKE_OFF 0



struct _mr_gnss_status {
    double longitude;
    double latitude; 
    double height;   
    float speed;     
    int loc_index;   

    int sat_num;     

    float yaw;       
    float pitch;     
    float roll;      

    time_t time_last;
};
typedef struct _mr_gnss_status mr_gnss_status;



struct _mr_chassis_status {
    float left_speed;
    float right_speed;
    
    unsigned char putter_state; 
    unsigned char cutter_state; 
    unsigned char auto_mode;    
    unsigned char brake_state;
    
    unsigned char safe_edge_front;
    unsigned char safe_edge_back;
    unsigned char safe_edge_left;
    unsigned char safe_edge_right;

    unsigned char soc;
    unsigned char battery_charge_state;
    char fault_code[8];

    float battery_volt;

    float sonar_front_left;
    float sonar_front_center;
    float sonar_front_right;
    float sonar_rear_left;
    float sonar_rear_center;
    float sonar_rear_right;
    
    uint64 time_last;  
};

typedef struct _mr_chassis_status mr_chassis_status;


struct _mr_chassis_control
{
    float left_speed;         
    float right_speed;        
    int brake_set;            
    int putter_set;           
    int cutter_set;           
    char system_status[8]; 
    uint64 time_last;    
};
typedef struct _mr_chassis_control mr_chassis_control;


typedef cv::Point mr_local_point;
typedef cv::Point2d mr_point;





#define DIR_INDEX_UNKNOWN       255 
#define DIR_INDEX_RIGHT         0
#define DIR_INDEX_RIGHT_FRONT   1
#define DIR_INDEX_FRONT         2
#define DIR_INDEX_LEFT_FRONT    3
#define DIR_INDEX_LEFT          4
#define DIR_INDEX_LEFT_BACK     5
#define DIR_INDEX_BACK          6
#define DIR_INDEX_RIGHT_BACK    7



class mr_pose {
public:
    double x;
    double y;
    double angle;
public:
    mr_pose(double x, double y, double angle):x(x),y(y),angle(angle){} 
    mr_pose():x(0), y(0), angle(0) {}
    ~mr_pose(){}
    mr_pose& operator = (const mr_pose& pose) {
        this->x = pose.x;
        this->y = pose.y;
        this->angle = pose.angle;
        return *this;
    }  
};

class mr_cmd {
public:
    double param1;
    double param2;
public:
    mr_cmd(double param1, double param2):param1(param1),param2(param2){} 
    mr_cmd():param1(0), param2(0) {}
    ~mr_cmd(){}
    mr_cmd& operator = (const mr_cmd& cmd) {
        this->param1 = cmd.param1;
        this->param2 = cmd.param2;
        return *this;
    }  
};


struct PointHash {
    std::size_t operator()(const mr_local_point& point) const {
        std::size_t h1 = std::hash<int>()(point.x);
        std::size_t h2 = std::hash<int>()(point.y);

        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

struct PointEqual {
    bool operator()(const mr_local_point& lhs, const mr_local_point& rhs) const {

        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};

using LocalPointHashMap = std::unordered_set<mr_local_point, PointHash, PointEqual>;


template <typename ValueType>
using LocalPointHashMapExt = std::unordered_map<mr_local_point, ValueType, PointHash, PointEqual>;

#endif