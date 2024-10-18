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

#ifndef __KALMAN_FILTER2_H__
#define __KALMAN_FILTER2_H__

#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter2 
{
private:
    double deltaT;
    
    Eigen::Vector4d state;     
    Eigen::Matrix4d matrix_P;  
    Eigen::Matrix4d matrix_Q;  
    Eigen::Matrix4d matrix_R;  
    Eigen::Matrix4d matrix_A;
    Eigen::Matrix4d matrix_F;
    Eigen::Matrix4d matrix_H;

public:
    KalmanFilter2();
    ~KalmanFilter2();
    void initialize(Eigen::Vector4d& state, Eigen::Matrix4d& measure_noise, Eigen::Matrix4d& process_noise, double deltaT);
    void predict(); 
    void update(Eigen::Vector4d& measurement);
    void getPost(Eigen::Vector4d& post);
};


#endif