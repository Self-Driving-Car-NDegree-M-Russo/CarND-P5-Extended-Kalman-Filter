#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
   * KF Prediction step
   *
   * The prediction step is the same for KF and EKF
   * NOTE: we assume process noise mean = 0, hence there is no input vector u
   */

  // Predicted State
  x_ = F_ * x_;

  MatrixXd Ft_ = F_.transpose();

  // Predicted P
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * KF state update
   */

   // measurement update
   VectorXd y_ = z - H_ * x_;

   MatrixXd Ht_ = H_.transpose();
   MatrixXd S_ = H_ * P_ * Ht_ + R_;
   MatrixXd Si_ = S_.inverse();
   MatrixXd K_ =  P_ * Ht_ * Si_;

   //Identity matrix
   MatrixXd I = MatrixXd::Identity(4, 4);

   // Updated state
   x_ = x_ + (K_ * y_);

   // Updated P
   //P_ = (I - K_ * H_) * P_;
   P_ -= K_ * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * EKF state update
   */

   // measurement update
   // nonlinear updates
   float px = x_[0];
   float py = x_[1];
   float vx = x_[2];
   float vy = x_[3];

   float h0 = sqrt((px*px)+(py*py));

   float h1 = atan2(py,px);
   // NOTE: the following if/else helps in dealing with the position from positive to negative y.
   // This condition has, in fact, shown some non-smooth behavior in simulations
   //if (abs(py)>0.05){
  //   h1 = atan2(py,px);
   //} else {
  //   h1 = atan2(0.0,px);
  // }

   float h2 = ((px*vx)+(py*vy))/h0;

   VectorXd y_;
   y_ = VectorXd(3);
   VectorXd z_pred;
   z_pred = VectorXd(3);

   z_pred << h0, h1, h2;
   y_ = z - z_pred;

   // Normalize y_(1)
   NormalizeAngle(y_(1));

   //float y0 = z[0] - h0;
   //float y1 = z[1] - h1;
   //float y2 = z[2] - h2;

   //y_ << y0, y1, y2;

   MatrixXd Ht_ = H_.transpose();
   MatrixXd S_ = H_ * P_ * Ht_ + R_;
   MatrixXd Si_ = S_.inverse();
   MatrixXd K_ =  P_ * Ht_ * Si_;

   //Identity matrix
   MatrixXd I = MatrixXd::Identity(4, 4);

   // Updated state
   x_ = x_ + (K_ * y_);

   // Updated P
   P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::NormalizeAngle(double& phi)
{
    phi = atan2(sin(phi), cos(phi));
}
