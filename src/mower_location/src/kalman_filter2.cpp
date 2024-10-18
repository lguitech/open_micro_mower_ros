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

#include "kalman_filter2.h"

KalmanFilter2::KalmanFilter2()
{

}

KalmanFilter2::~KalmanFilter2()
{

}
void KalmanFilter2::initialize(Eigen::Vector4d& state, Eigen::Matrix4d& measure_noise,
     Eigen::Matrix4d& process_noise, double deltaT)
{
    this->state = state;
    this->matrix_P = Eigen::Matrix4d::Identity();
    this->matrix_R = measure_noise;
    this->matrix_Q = process_noise;
    this->deltaT = deltaT;

    this->matrix_H << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;  

}


void KalmanFilter2::predict()
{
    double vk = state(2);
    double psik = state(3);
    
    matrix_A << 1,  0,  deltaT * cos(psik),  0,
                0,  1,  deltaT * sin(psik),  0,
                0,  0,       1,              0,
                0,  0,       0,              1;

    matrix_F << 1,  0,  deltaT * cos(psik),   -deltaT * vk * sin(psik),
                0,  1,  deltaT * sin(psik),    deltaT * vk * cos(psik),
                0,  0,         1,                       0,
                0,  0,         0,                       1;

    state = matrix_A * state;
    matrix_P = matrix_F * matrix_P * matrix_F.transpose() + matrix_Q;    
            
}

void KalmanFilter2::update(Eigen::Vector4d& measurement)
{

    auto K = matrix_P * matrix_H.transpose() * (matrix_H * matrix_P * matrix_H.transpose() + matrix_R).inverse();

    state = state + K * (measurement - matrix_H * state);
    
    matrix_P = (Eigen::Matrix4d::Identity() - K * matrix_H) * matrix_P;

}

void KalmanFilter2::getPost(Eigen::Vector4d& post)
{
    post = state;
}
