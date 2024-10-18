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

#ifndef __KALMAN_FILTER1_H__
#define __KALMAN_FILTER1_H__

#include <Eigen/Dense>

using namespace Eigen;
class KalmanFilter1 
{
private:
    static constexpr double Tc = 300;
    double deltaT;

    Eigen::Vector2d state;     
    Eigen::Vector2d matrix_U;  
    Eigen::Vector2d matrix_Z;  

    Eigen::Matrix2d matrix_A;
    Eigen::Matrix2d matrix_B;  
    Eigen::Matrix2d matrix_H;

    Eigen::Matrix2d matrix_P;  
    Eigen::Matrix2d matrix_R;  
    Eigen::Matrix2d matrix_Q;  

public:
    KalmanFilter1();
    ~KalmanFilter1();
    void initialize(double psi, double measure_noise, double process_noise, double deltaT);
    void reset_state(double value);
    void predict(double dPsiGyro); 
    void update(double measurement);

    double getPost();
};


#endif