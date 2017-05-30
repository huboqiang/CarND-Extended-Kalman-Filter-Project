#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const float PI = 3.1415927;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0,      0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0,      0,
    0,    0.0009, 0,
    0,    0,      0.09;
    
    /**
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    noise_ax = 9.0;
    noise_ay = 9.0;
    KalmanFilter ekf;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {
}

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
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        //ekf_.x_ << 1, 1, 1, 1;
        
        ekf_.P_ = MatrixXd(4,4);
        ekf_.P_ <<  1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
        
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            float ro     = measurement_pack.raw_measurements_(0);
            float theta  = measurement_pack.raw_measurements_(1);
            float ro_dot = measurement_pack.raw_measurements_(2);
            
            float px = ro * cos(theta);
            float py = ro * sin(theta);
            float vx = ro_dot * cos(theta);
            float vy = ro_dot * sin(theta);
            
            ekf_.x_ << px, py, vx, vy;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
            
            
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
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
    float dt = measurement_pack.timestamp_ - previous_timestamp_;
    
    //  ms to s
    dt /= 1000000.0;
    
    float dt2 = dt*dt;
    float dt3 = dt*dt*dt;
    float dt4 = dt*dt*dt*dt;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << dt4/4*noise_ax,0,              dt3/2*noise_ax,0,
    0,             dt4/4*noise_ay, 0,             dt3/2*noise_ay,
    dt3/2*noise_ax,0,              dt2*noise_ax,  0,
    0,             dt3/2*noise_ay, 0,             dt2*noise_ay;
    
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        VectorXd z = VectorXd(3);
        z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],
        measurement_pack.raw_measurements_[2];
        
        
        /*
         Perform Normalizing Angles.
         In C++, atan2() returns values between -pi/2 and pi/2.
         When calculating phi in y = z - h(x) for radar measurements,
         the resulting angle phi in the y vector should be adjusted,
         so that it is between -pi/2 and pi/2.
         The Kalman filter is expecting small angle values between the range -pi/2 and pi/2.
         
         So when working in radians, I just add π or subtract π until the angle is within the desired range.
         
         Lastly, WHY YOU SAY  "C++, atan2() returns values between -pi and pi" IN YOUR TIPS? I was puzzled for HOURS
         until I searched [wikipedia](https://en.wikipedia.org/wiki/Inverse_trigonometric_functions) and found this
         error.
         */
        while(z(1) > PI/2){
            z(1) -= PI;
        }
        while(z(1) < -1 * PI/2){
            z(1) += PI;
        }
        
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.H_ = Hj_;
        
        if (!Hj_.isZero(0)){
            ekf_.UpdateEKF(z);
        }
        
    }
    else {
        // Laser updates
        ekf_.H_ = MatrixXd(2,4);
        ekf_.H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
        
        VectorXd z = VectorXd(2);
        ekf_.R_ = R_laser_;
        z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
        ekf_.Update(z);
    }
    
    // print the output
    cout << "Measurement:" << measurement_pack.sensor_type_ << endl;
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
