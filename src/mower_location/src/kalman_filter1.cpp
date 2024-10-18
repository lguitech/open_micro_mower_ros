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

#include "kalman_filter1.h"

KalmanFilter1::KalmanFilter1()
{

}

KalmanFilter1::~KalmanFilter1()
{


}

void KalmanFilter1::initialize(double psi, double measure_noise, double process_noise, double deltaT)
{
    state(0) = psi;
    state(1) = 0;

    matrix_P = Eigen::Matrix2d::Identity();
    matrix_Q = Eigen::Matrix2d::Identity() * process_noise;
    matrix_R = Eigen::Matrix2d::Identity() * measure_noise;

    this->deltaT = deltaT;     

    matrix_A << 1, 1,
                0, 1 - 1/Tc * deltaT;            

    matrix_H = Eigen::Matrix2d::Identity();
    matrix_B = Eigen::Matrix2d::Identity();
}

void KalmanFilter1::reset_state(double value)
{
    state(0) = value;
}

void KalmanFilter1::predict(double dPsiGyro)
{
    matrix_U(0) = dPsiGyro;
    matrix_U(1) = 0;

    // Prediction
    state = matrix_A * state + matrix_B * matrix_U;
    matrix_P = matrix_A * matrix_P * matrix_A.transpose() + matrix_Q;     
}

void KalmanFilter1::update(double measurement)
{
    matrix_Z << measurement, 0;

    auto K = matrix_P * matrix_H.transpose() * (matrix_H * matrix_P * matrix_H.transpose() + matrix_R).inverse();

    state = state + K * (matrix_Z - matrix_H * state);

    matrix_P = (Eigen::Matrix2d::Identity() - K * matrix_H) * matrix_P;
}

double KalmanFilter1::getPost()
{
    return state(0);
}