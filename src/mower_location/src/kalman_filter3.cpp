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

#include "kalman_filter3.h"

KalmanFilter3::KalmanFilter3()
{

}

KalmanFilter3::~KalmanFilter3()
{

}

void KalmanFilter3::initialize(double utmx, double utmy, 
        double measure_noise, double process_noise, double deltaT)
{
    this->state << utmx, utmy;

    this->matrix_A = Eigen::Matrix2d::Identity();
    this->matrix_B = Eigen::Matrix2d::Identity();
    this->matrix_H = Eigen::Matrix2d::Identity();

    this->matrix_P = Eigen::Matrix2d::Identity();
    this->matrix_R = Eigen::Matrix2d::Identity() * measure_noise;
    this->matrix_Q = Eigen::Matrix2d::Identity() * process_noise;

    this->deltaT = deltaT;
}

void KalmanFilter3::predict(double speed, double psi)
{
    matrix_U << speed * deltaT * cos(psi),
                speed * deltaT * sin(psi);

    state = matrix_A * state + matrix_B * matrix_U;               

    matrix_P = matrix_A * matrix_P * matrix_A.transpose() + matrix_Q;    
}

void KalmanFilter3::update(double utmx, double utmy)
{
    matrix_Z << utmx, utmy;

    auto K = matrix_P * matrix_H.transpose() * (matrix_H * matrix_P * matrix_H.transpose() + matrix_R).inverse();

    state = state + K * (matrix_Z - matrix_H * state);
    
    matrix_P = (Eigen::Matrix2d::Identity() - K * matrix_H) * matrix_P;
}

void KalmanFilter3::getPost(double& utmx, double& utmy)
{
    utmx = this->state(0);
    utmy = this->state(1);
}
