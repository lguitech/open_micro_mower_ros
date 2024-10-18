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

#ifndef INS_ERA_TYPES_H
#define INS_ERA_TYPES_H

#include <string>
#include <vector>
#include <list>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>


using namespace std;


typedef struct _EraRawGnss 
{
    unsigned int timestamp;

    unsigned char  loc_index;
    unsigned char heading_index;
    unsigned char front_sat;
    unsigned char rear_sat;

    double lon;
    double lat;

    float speed; 
    float height;
    float heading;
    float pitch;  

} EraRawGnss;

typedef struct _EraRawIMU
{
    unsigned int timestamp;
    float pitch;    
    float roll;     
    float yaw;      
    float ax;       
    float ay;       
    float az;       
    float gx;       
    float gy;       
    float gz;       
    float temp;     
} EraRawIMU;

typedef struct _EraRawGGA 
{
    unsigned int timestamp;

    unsigned char loc_index;
    unsigned char  sat_num;
    float hdop;

    double lon;
    double lat;

    float ant_height;
    float ellip_height;
    unsigned short age;
    unsigned short base_index;

} EraRawGGA;



#endif
