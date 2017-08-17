#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using namespace Eigen;

class KalmanFilter {
public:
    
    // state vector
    Vector4d x_;
    // state transition matrix
    Matrix<double, 4, 4> F_;
    // state covariance matrix
    Matrix<double, 4, 4> P_;
    // process covariance matrix
    Matrix<double, 4, 4> Q_;
    // laser measurement covariance matrix
    Matrix<double, 2, 2> R_laser_;
    // radar measurement covariance matrix
    Matrix<double, 3, 3> R_radar_;
    // laser measurement matrix
    Matrix<double, 2, 4> H_laser_;
    // Jacobian measurement matrix
    Matrix<double, 3, 4> Hj_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
