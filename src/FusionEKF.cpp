#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measuremet matrix - radar
  //NOTE: this will be overwritten by the calculation of the Jacobian
  Hj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

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
    
    /**
     * In case the filter is not initialized, it will have to be based on the current state.
     * The matrices to use will be those defined in the constructor, plus the process and covariance
     * matrices, that for the initialization will be put to:
     *  - All zeros for Q
     *  - The diagonal matrix for F
     *  _ A high value for P
     */

    // first measurement
    cout << "EKF: " << endl;

    // Declaration of variables
    float px = 0.0;
    float py = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    VectorXd x_in;
    MatrixXd Q_in;
    MatrixXd F_in;
    MatrixXd P_in;

    // Set dimensions
    x_in = VectorXd(4);
    Q_in = MatrixXd(4,4);
    F_in = MatrixXd(4,4);
    P_in = MatrixXd(4,4);

    // Initialize Q matrix matrix to 0
    Q_in << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;

    // Initialize F matrix to identity
    F_in << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    // Initialize P matrix with a big value
    P_in << 1000.0, 0.0, 0.0, 0.0,
            0.0, 1000.0, 0.0, 0.0,
            0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 1000.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Initialize state starting from radar measurements (rho, theta) and convert from polar to cartesian
      // Initial velocity is assumed = 0

      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];

      px = rho * cos(theta);
      py = rho * sin(theta);

      // Initial state
      x_in << px, py, vx, vy;

      // Initialize
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state with current laser measurements (px, py)
      // Initial velocity is assumed = 0

      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

      // Initial state
      x_in << px, py, vx, vy;

      // Initialize
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

    }

    // Update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * First the state transition matrix F is updated according to the new elapsed time.
   * Then the process noise covariance matrix is also updated, still based on time.
   * Values of noise_ax = 9 and noise_ay = 9 are used for the Q matrix.
   */

  // read current timestamp
  long long current_timestamp_ = measurement_pack.timestamp_;

  // calculate elapsed time
  // NOTE: timestamps are in microseconds, while elapsed time needs to be in seconds
  float elapsed_time = (current_timestamp_ - previous_timestamp_)/1000000.0;

  float noise_ax = 81.0; // sigma_square ax
  float noise_ay = 81.0; // sigma_square ay

  // Introduction of Q, F
  MatrixXd Q_p;
  MatrixXd F_p;
  Q_p = MatrixXd(4,4);
  F_p = MatrixXd(4,4);

  // helping coefficients to calculate Q
  float c1 = (pow(elapsed_time,4.0))/4.0;
  float c2 = (pow(elapsed_time,3.0))/2.0;
  float c3 = pow(elapsed_time, 2.0);

  // Update Q, F
  Q_p << (c1*noise_ax), 0.0, (c2*noise_ax), 0.0,
         0.0, (c1*noise_ay), 0.0, (c2*noise_ay),
         (c2*noise_ax), 0.0, (c3*noise_ax), 0.0,
         0.0, (c2*noise_ay), 0.0, (c3*noise_ay);

  F_p << 1.0, 0.0, elapsed_time, 0.0,
         0.0, 1.0, 0.0, elapsed_time,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;


  ekf_.F_ = F_p;
  ekf_.Q_ = Q_p;

  // Predict
  ekf_.Predict();

  //cout << "PREDICTED x_ = " << ekf_.x_ << endl;
  //cout << "PREDICTED P_ = " << ekf_.P_ << endl;

  /**
   * Update
   */

  /**
   * Based on the sensor type the appropriate update step is performed, and then the state and covariance matrix are
   * upadted.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Nonlinear measurements require EKF update

    // Calculate and update Jacobian
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    // Set H and R appropriately
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;

    // Update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    // Simple KF update mechanism can be used for linear measurements

    // Set H and R appropriately
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    // Update
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // Update timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
