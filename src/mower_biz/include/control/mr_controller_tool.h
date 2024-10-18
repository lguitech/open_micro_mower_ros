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

#ifndef __MR_CONTROLLER_TOOL_H__
#define __MR_CONTROLLER_TOOL_H__

#include <vector>
#include "mr_navi_types.h"
#include "mr_util.h"
#include "mr_coord.h"

#define ROTATE_THRESHOLD (M_PI/12)
//#define ROTATE_THRESHOLD (M_PI/18)
#define LOOKAHEAD_THRESHOLD (M_PI/4)

class MR_ControllerTool {
private:
    double MAX_LINEAR_SPEED = 0.3;
    double MAX_ANGULAR_SPEED = M_PI/4;
    double wheel_distance;
private:
    double calc_coefficient1(double alpha);
    double calc_coefficient2(double alpha);
    double calc_coefficient3(double alpha);
    
    void calc_by_angular_speed(double alpha, double radius, double& speed_faster, double& speed_slower);    
    void calc_by_linear_speed(double alpha, double radius, double& speed_faster, double& speed_slower);    
    void calc_diff_speed(double alpha, double radius, double& speed_faster, double& speed_slower);    
public:
    MR_ControllerTool();
    MR_ControllerTool(double wheel_distance);
    ~MR_ControllerTool();

    void set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance);
    void rotate_left_max_speed(double alpha, mr_cmd& param);
    void rotate_right_max_speed(double alpha, mr_cmd& param);
    void straight_forward_max_speed(mr_cmd& param);
    void straight_forward_mid_speed(mr_cmd& param);


    void rotate_left(double alpha, double radius, mr_cmd& param);
    void rotate_right(double alpha, double radius, mr_cmd& param);
};

#endif