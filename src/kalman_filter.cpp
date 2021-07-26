#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
    * DONE: predict the state
    */
    // The prediction step always uses a linear model
    // regardless of the sensor input type, so we can use
    // the regular Kalman filter equations
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    * DONE: update the state by using Kalman Filter equations
    */
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    * DONE: update the state by using Extended Kalman Filter equations
    */
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    double rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
    double theta = atan2(x_[1], x_[0]);
    double row_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;

    VectorXd h = VectorXd(3);
    h << rho, theta, row_dot;
    VectorXd y = z - h;

    // Restrict phi to [-PI, PI]
    if (y(1) > M_PI)
    {
        y(1) -= M_PI * 2.0f;
    }
    else if (y(1) < -M_PI)
    {
        y(1) += M_PI * 2.0f;
    }

    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}
