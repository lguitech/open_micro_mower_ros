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

#ifndef __MR_UTIL_H__
#define __MR_UTIL_H__

#include <cmath>
#include <vector>
#include <string>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

namespace mower {

double PointDistance(const double x1, const double y1, 
            const double x2, const double y2);

double PointDistanceSquare(const double x1, const double y1,
            const double x2, const double y2);

double deg2rad(double deg);

double rad2deg(double rad);

int byte2int(const unsigned char* buffer);

void int2byte(int input, unsigned char* output);

bool split_string(std::string strSentence, std::vector<std::string>& vec, char tag);


}



#ifdef __cplusplus
}
#endif



#endif