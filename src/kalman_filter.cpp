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

  cout << "Init in" << endl;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  cout << "Init out" << endl;
}

void KalmanFilter::Predict() {
  /**
   * KF Prediction step
   *
   * The prediction step is the same for KF and EKF
   * NOTE: we assume process noise mean = 0, hence there is no input vector u
   */

  //cout << "Predict 0" << endl;
  x_ = F_ * x_;

  //cout << "Predict 1" << endl;
  MatrixXd Ft_ = F_.transpose();

  //cout << "Predict 2" << endl;

  P_ = F_ * P_ * Ft_ + Q_;
  //cout << "Predict 3" << endl;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * KF state update
   */

   // measurement update
   //cout << "Update 0" << endl;
   VectorXd y_ = z - H_ * x_;

   //cout << "Update 1" << endl;
   MatrixXd Ht_ = H_.transpose();

   //cout << "Update 2" << endl;
   MatrixXd S_ = H_ * P_ * Ht_ + R_;

   //cout << "Update 3" << endl;
   MatrixXd Si_ = S_.inverse();

   //cout << "Update 4" << endl;
   MatrixXd K_ =  P_ * Ht_ * Si_;

   //cout << "Update 5" << endl;

   //Identity matrix
   MatrixXd I = MatrixXd::Identity(4, 4);

   // new state
   x_ = x_ + (K_ * y_);
   //cout << "Update 6" << endl;

   P_ = (I - K_ * H_) * P_;
   //cout << "Update 7" << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

   // measurement update
   //cout << "ext Update 0" << endl;

   // nonlinear updates
   float px = x_[0];
   float py = x_[1];
   float vx = x_[2];
   float vy = x_[3];

   float h0 = sqrt((px*px)+(py*py));
   cout << "h0: " << h0 << endl;

   float h1 = 0.0;
   if (abs(py)>0.05){
     h1 = atan2(py,px);
   } else {
     h1 = atan2(0.0,px);
   }

   cout << "h1: " << h1 << endl;
   float h2 = ((px*vx)+(py*vy))/h0;
   cout << "h2: " << h2 << endl;

   VectorXd y_;
   y_ = VectorXd(3);

   float y0 = z[0] - h0;
   float y1 = z[1] - h1;
   float y2 = z[2] - h2;

   cout << "y0: " << y0 << endl;
   cout << "y1: " << y1 << endl;
   cout << "y2: " << y2 << endl;

   y_ << y0, y1, y2;

   //cout << "ext Update 1" << endl;
   MatrixXd Ht_ = H_.transpose();

   //cout << "ext Update 2" << endl;
   MatrixXd S_ = H_ * P_ * Ht_ + R_;

   //cout << "ext Update 3" << endl;
   MatrixXd Si_ = S_.inverse();

   //cout << "ext Update 4" << endl;
   MatrixXd K_ =  P_ * Ht_ * Si_;

   //cout << "ext Update 5" << endl;

   //Identity matrix
   MatrixXd I = MatrixXd::Identity(4, 4);

   // new state
   x_ = x_ + (K_ * y_);
   //cout << "ext Update 6" << endl;

   P_ = (I - K_ * H_) * P_;
   //cout << "ext Update 7" << endl;
}
