#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    /**
     TODO:
     * predict the state
     */
    
    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Kalman Filter equations
     */
    VectorXd z_pred = H_laser_ * x_;
    VectorXd y = z - z_pred;
    
    MatrixXd S = H_laser_* P_ * H_laser_.transpose() + R_laser_;
    MatrixXd K = P_ * H_laser_.transpose() * S.inverse();
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */
    
    // Convert predicted values from cartesian to polar
    double p_x = x_(0);
    double p_y = x_(1);
    double v_x = x_(2);
    double v_y = x_(3);
    
    
    double ro = sqrt(p_x*p_x + p_y*p_y);
    double theta = atan2(p_y, p_x);
    double ro_dot = (p_x*v_x+p_y*v_y)/(ro+1e-5);
    
    VectorXd z_pred(3,1);
    z_pred << ro, theta, ro_dot;
    
    // Error
    VectorXd y = z - z_pred;
    
    //normalizing theta
    while (y[1] > M_PI) {
        y[1] = y[1]-2*M_PI;
    }
    while (y[1] < -M_PI) {
        y[1] = y[1]+2*M_PI;
    }
    
    MatrixXd S = Hj_* P_ * Hj_.transpose() + R_radar_;
    MatrixXd K = P_ * Hj_.transpose() * S.inverse();
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
}
