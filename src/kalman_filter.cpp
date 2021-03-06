#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateCommon(const VectorXd &y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - H_ * x_;

    // Update state
    UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Convert x_ to polar
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  VectorXd h = VectorXd(3);
  double sqrt_py2px2 = sqrt((pow(px,2) + pow(py,2)));

  // Check for division by zero
  if(sqrt_py2px2 < 0.0001) {
      // Increment px and py by a small amount
      px += 0.0001;
      py += 0.0001;

      // Re-evaluate sqrt_py2px2
      sqrt_py2px2 = sqrt((pow(px,2) + pow(py,2)));
  }

  h << sqrt_py2px2,
       atan2(py,px),
       (px*vx + py*vy) / sqrt_py2px2;

  VectorXd y = z - h;

  // Check that the angle y(1) is > -pi and < pi, else convert to between -pi and pi by removing multiples of 2 *pi
  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
        y(1) -= 2 * M_PI;
    } else {
        y(1) += 2 * M_PI;
    }
  }

    // Update state
    UpdateCommon(y);
}

