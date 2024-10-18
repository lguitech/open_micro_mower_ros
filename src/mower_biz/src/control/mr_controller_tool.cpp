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
#include "mr_controller_tool.h"


MR_ControllerTool::MR_ControllerTool(double wheel_distance)
{
    this->wheel_distance = wheel_distance;
}

MR_ControllerTool::MR_ControllerTool()
{

}

MR_ControllerTool::~MR_ControllerTool()
{

}

void MR_ControllerTool::set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance)
{
    this->MAX_LINEAR_SPEED = max_linear_speed;
    this->MAX_ANGULAR_SPEED = max_angular_speed;
    this->wheel_distance = wheel_distance;    
}

double MR_ControllerTool::calc_coefficient1(double alpha)
{
    return MAX_ANGULAR_SPEED / 2 * (1 + (alpha - ROTATE_THRESHOLD)/(M_PI - ROTATE_THRESHOLD));
}
double MR_ControllerTool::calc_coefficient2(double alpha)
{
    return MAX_ANGULAR_SPEED / 4 * (1 + (alpha/ROTATE_THRESHOLD));
}

double MR_ControllerTool::calc_coefficient3(double alpha)
{
    return MAX_LINEAR_SPEED / (1 + (2 * alpha/ROTATE_THRESHOLD));
}

void MR_ControllerTool::rotate_left_max_speed(double alpha, mr_cmd& param)
{
    double linear_speed = calc_coefficient1(alpha) * wheel_distance / 2;
    
    linear_speed = std::min(linear_speed, MAX_LINEAR_SPEED); 

    param.param1 = -linear_speed;
    param.param2 = linear_speed;
}


void MR_ControllerTool::rotate_right_max_speed(double alpha, mr_cmd& param)
{
    double linear_speed = calc_coefficient1(alpha) * wheel_distance / 2;

    linear_speed = std::min(linear_speed, MAX_LINEAR_SPEED);

    param.param1 = linear_speed;
    param.param2 = -linear_speed;
}

void MR_ControllerTool::straight_forward_max_speed(mr_cmd& param)
{
    param.param1 = MAX_LINEAR_SPEED;
    param.param2 = MAX_LINEAR_SPEED;
}

void MR_ControllerTool::straight_forward_mid_speed(mr_cmd& param)
{
    param.param1 = MAX_LINEAR_SPEED / 2.0;
    param.param2 = MAX_LINEAR_SPEED / 2.0;
}

void MR_ControllerTool::calc_by_angular_speed(double alpha, double radius, double& speed_faster, double& speed_slower)
{
    double coefficient = calc_coefficient2(alpha);
    speed_slower = (radius - wheel_distance / 2) * coefficient;
    speed_faster = (radius + wheel_distance / 2) * coefficient;

}

void MR_ControllerTool::calc_by_linear_speed(double alpha, double radius, double& speed_faster, double& speed_slower)
{
    double c_value = (radius - wheel_distance / 2) / (radius + wheel_distance / 2);
    speed_faster = (2 * calc_coefficient3(alpha)) / (c_value + 1);
    speed_slower = c_value * speed_faster;
}

void MR_ControllerTool::calc_diff_speed(double alpha, double radius, double& speed_faster, double& speed_slower)
{
    if (radius <= wheel_distance) {
        calc_by_angular_speed(alpha, radius, speed_faster, speed_slower);
    }
    else {
        calc_by_linear_speed(alpha, radius, speed_faster, speed_slower);
    }
}


void MR_ControllerTool::rotate_left(double alpha, double radius, mr_cmd& param)
{
    double speed_faster, speed_slower;
    calc_diff_speed(alpha, radius, speed_faster, speed_slower);
    param.param1 = speed_slower;
    param.param2 = speed_faster;
}

void MR_ControllerTool::rotate_right(double alpha, double radius, mr_cmd& param)
{
    double speed_faster, speed_slower;
    calc_diff_speed(alpha, radius, speed_faster, speed_slower);
    param.param1 = speed_faster;
    param.param2 = speed_slower;
}

