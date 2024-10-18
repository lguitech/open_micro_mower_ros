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

#include "mr_sonarfilter.h"

mr_class_sonar_filter::mr_class_sonar_filter()
{

}

mr_class_sonar_filter::~mr_class_sonar_filter()
{

}


float mr_class_sonar_filter::addData(float new_value) 
{

    if (window.size() == window_size) {
        window.pop_front();
    }
    window.push_back(new_value);


    std::vector<float> sorted_window(window.begin(), window.end());
    std::nth_element(sorted_window.begin(), sorted_window.begin() + sorted_window.size() / 2, sorted_window.end());


    if (sorted_window.size() % 2 != 0) {
        return sorted_window[sorted_window.size() / 2];
    }

    else {
        std::nth_element(sorted_window.begin(), sorted_window.begin() + sorted_window.size() / 2 - 1, sorted_window.end());
        float middle1 = sorted_window[sorted_window.size() / 2];
        float middle2 = sorted_window[sorted_window.size() / 2 - 1];
        return (middle1 + middle2) / 2.0;
    }
}