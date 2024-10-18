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

#include "mr_util.h"

namespace mower {

double PointDistance(const double x1, const double y1, const double x2, const double y2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy); 
}

double PointDistanceSquare(const double x1, const double y1, const double x2, const double y2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return dx * dx + dy * dy;
}

double convert_to_degree(double value) 
{
    int ipart = (int)value;
    int deg = ipart / 100;
    double min = value - (int)(ipart / 100) * 100;
    return deg + min / 60.0;
}

double deg2rad(double deg)
{
    return deg * M_PI / 180;
}

double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

int byte2int(const unsigned char* buffer)
{

    return (buffer[3] << 24) + (buffer[2] << 16) +  (buffer[1] << 8) + buffer[0];
}

void int2byte(int input, unsigned char* output)
{
    output[0] = input & 0xff;
    output[1] = (input >> 8) & 0xff;
    output[2] = (input >> 16) & 0xff;
    output[3] = (input >> 24) & 0xff;
}


bool split_string(std::string strSentence, std::vector<std::string>& vec, char tag)
{
    const char* buffer = strSentence.c_str();
    int len_total = strSentence.length();
    
    int pos_prev = -1;
    int count = 0;
    char sztemp[1024];
    for (int i=0; i<len_total; i++) {
        if (buffer[i] == tag) {
            count += 1;
            int len = i-pos_prev-1;
            if (len > 1024) {
                printf("split_string, too long splitted string\r\n");
                return false;
            }
            strncpy(sztemp, buffer + pos_prev + 1, len);
            sztemp[len] = 0;
            vec.push_back(std::string(sztemp));
            pos_prev = i; 
        }    
    }
    //last one
    int len = len_total - pos_prev - 1;
    if (len > 1024) {
        printf("split_string, too long splitted string\r\n");
        return false;
    }
    strncpy(sztemp, buffer + pos_prev + 1, len);
    sztemp[len] = 0;
    vec.push_back(std::string(sztemp));

    return true;
}



}