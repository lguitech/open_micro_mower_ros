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

#ifndef __KALMAN_FILTER3_H__
#define __KALMAN_FILTER3_H__

#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter3 
{
private:
    double deltaT;
    
    Eigen::Vector2d state;    
    Eigen::Vector2d matrix_U; 
    Eigen::Vector2d matrix_Z; 

    Eigen::Matrix2d matrix_A;
    Eigen::Matrix2d matrix_B;
    Eigen::Matrix2d matrix_H;

    Eigen::Matrix2d matrix_P; 
    Eigen::Matrix2d matrix_Q; 
    Eigen::Matrix2d matrix_R; 

public:
    KalmanFilter3();
    ~KalmanFilter3();
    void initialize(double utmx, double utmy, double measure_noise, double process_noise, double deltaT);
    void predict(double speed, double psi); 
    void update(double utmx, double utmy);
    void getPost(double& utmx, double& utmy);
};

#endif