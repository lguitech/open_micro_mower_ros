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

#ifndef __MR_SONAR_FILTER_H__
#define __MR_SONAR_FILTER_H__

#include <ros/ros.h>
#include "mr_navi_types.h"

class mr_class_sonar_filter {
private:
    const size_t window_size = 5;
    std::deque<float> window;
public:
    mr_class_sonar_filter();
    ~mr_class_sonar_filter();
    float addData(float new_value); 
};

#endif

