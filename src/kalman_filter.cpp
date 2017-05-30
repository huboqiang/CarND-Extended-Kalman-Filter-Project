#include "kalman_filter.h"
#include <math.h>
#include <iostream>

const float PI = 3.1415927;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
        x_ = x_in;
        P_ = P_in;
        F_ = F_in;
        H_ = H_in;
        R_ = R_in;
        Q_ = Q_in;
}

void KalmanFilter::Predict() {
        /**
           TODO:
         * predict the state
         */

        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::RunY(const VectorXd &y){
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    x_  =  x_ + K*y;
    P_  = (I - K*H_)*P_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
        TODO:
      * update the state by using Kalman Filter equations
    */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    RunY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
        TODO:
       * update the state by using Extended Kalman Filter equations
    */

    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    float ro     = sqrt(px*px+py*py);
    
    //avoiding division by zero
    if (fabs(px) < 0.0001){
        px = 0.0001;
    }
    
    if (fabs(ro) < 0.0001) {
        ro = 0.0001;
    }
    
    float theta  = atan(py/px);
    float ro_dot = (vx*px+vy*py) / ro;
    
    
    //avoiding something like theta=-89 and z(1) = 89
    if (theta-z(1) > 0.9*PI){
        theta -= PI;
    }
    if (theta-z(1) < -0.9*PI){
        theta += PI;
    }
    VectorXd z2 = VectorXd(3);
    z2 << ro, theta, ro_dot;
    
    VectorXd y = z - z2;
    RunY(y);

}