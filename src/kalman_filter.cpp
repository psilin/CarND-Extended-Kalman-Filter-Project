#include "kalman_filter.h"
#include <math.h>

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
  x_ = F_ * x_/* + u == 0 */;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  return;
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd I = MatrixXd::Identity(4, 4);

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &Jac) {
  static float pi = 3.14159265358;

  MatrixXd I = MatrixXd::Identity(4, 4);

  VectorXd Hx(3);
  Hx << ::sqrt(x_(0)*x_(0) + x_(1)*x_(1)), ::atan2(x_(1), x_(0)), (x_(0)*x_(2) + x_(1)*x_(3)) / ::sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  VectorXd y = z - Hx;
  //normalize phi
  if (y(1) > pi) {
    while (y(1) > pi) {
      y(1) = y(1) - 2* pi;
    }
  }
  else if (y(1) < -pi) {
    while (y(1) < -pi) {
      y(1) = y(1) + 2* pi;
    }
  }

  MatrixXd Jact = Jac.transpose();
  MatrixXd S = Jac * P_ * Jact + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Jact * Si;

  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * Jac) * P_;
  return;
}
