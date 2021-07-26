#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
               0, 0.0009, 0,
              0, 0, 0.09;

    // measurement matrix H_laser_
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Initial state
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // State covariance P_
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    // Transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Process covariance matrix Q_
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

    // Measurement matrix
    ekf_.H_ = H_laser_;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
    * Initialization
    */
    if (!is_initialized_) {
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // Init state using raw_measurements_ << ro,theta, ro_dot;
            double ro = measurement_pack.raw_measurements_[0];
            double theta = measurement_pack.raw_measurements_[1];
            ekf_.x_[0] = ro * cos(theta);
            ekf_.x_[1] = ro * sin(theta);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // Init state using raw_measurements_ << px, py;
            ekf_.x_[0] = measurement_pack.raw_measurements_[0];
            ekf_.x_[1] = measurement_pack.raw_measurements_[1];
        }

        if (debug_) {
            cout << "First measurement received: " << endl;
            cout << "Px = " << ekf_.x_[0] << " Py = " << ekf_.x_[1] << endl;
        }

        // Store the first timestamp measurement
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
    * Update the state transition matrix F according to the new elapsed time.
    * Time is measured in seconds.
    */
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    /** Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
            0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
            dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    // Prediction
    ekf_.Predict();

    // Update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.H_ = Hj_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // print the output
    if (debug_)
    {
        cout << "x_ = " << ekf_.x_ << endl;
        cout << "P_ = " << ekf_.P_ << endl;
    }
}
