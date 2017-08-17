#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

using namespace std;
using namespace Eigen;
using std::vector;

class FusionEKF {
public:
    /**
     * Constructor.
     */
    FusionEKF();
    
    /**
     * Destructor.
     */
    virtual ~FusionEKF();
    
    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    
    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;
    
private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;
    
    // previous timestamp
    long long previous_timestamp_;
    
    // State
    Vector4d x_;
    
    Matrix<double, 4, 4> F_;
    Matrix<double, 4, 4> P_;
    Matrix<double, 4, 4> Q_;
    Matrix<double, 2, 2> R_laser_;
    Matrix<double, 3, 3> R_radar_;
    Matrix<double, 2, 4> H_laser_;
    Matrix<double, 3, 4> Hj_;
    
    float noise_ax;
    float noise_ay;
    
    // tool object used to compute Jacobian and RMSE
    Tools tools;
};

#endif /* FusionEKF_H_ */
