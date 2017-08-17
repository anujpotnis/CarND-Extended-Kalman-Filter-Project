#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;
    
    //laser measurement matrix
    ekf_.H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
    
    //measurement covariance matrix - laser
    ekf_.R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    //measurement covariance matrix - radar
    ekf_.R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    
    /**
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    
    // Initial Transition Matrix F_
    ekf_.F_ << 1,0,1,0,
    0,1,0,1,
    0,0,1,0,
    0,0,0,1;
    
    // State Covariance Matrix P_
    ekf_.P_ << 1,0,0,0,
    0,1,0,0,
    0,0,1000,0,
    0,0,0,1000;
    
    // set the acceleration noise components
    noise_ax = 9;
    noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    
    if (!is_initialized_) {
        /**
         TODO:
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        
        previous_timestamp_ = measurement_pack.timestamp_;
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            
            // This will not be used for Dataset 1
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    /**
     TODO:
     * Update the state transition matrix F according to the new elapsed time.
     - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    //Modify the F matrix so that the time is integrated
    
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    
    // set the acceleration noise components
    float noise_ax = 9;
    float noise_ay = 9;
    
    //set the process covariance matrix Q
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
    0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
    dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
    // Predict State
    ekf_.Predict();
    
    // Update State
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
        // Radar updates
         ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
         ekf_.Update(measurement_pack.raw_measurements_);
    }
}

