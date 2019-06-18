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
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


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

    // Initialize matrixes to 0
    Q_in << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;

    F_in << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    P_in << 1000.0, 0.0, 0.0, 0.0,
            0.0, 1000.0, 0.0, 0.0,
            0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 1000.0;

    // Initilaize velocity
    vx = 0.0;
    vy = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Initialize state starting from radar measurements (rho, theta) and convert from polar to cartesian
      // Initial velocity is assumed = 0

      //cout << "RADAR " << endl;

      float rho = measurement_pack.raw_measurements_[0];
      //cout << "rho: " << rho << endl;

      float theta = measurement_pack.raw_measurements_[1];
      //cout << "theta: " << theta << endl;

      px = rho * cos(theta);
      py = rho * sin(theta);

      // Initial state
      x_in << px, py, vx, vy;

      // Initialize
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state with current laser measurements (px, py) and 0 velocity (vx, vy)

      //cout << "LIDAR " << endl;

      px = measurement_pack.raw_measurements_[0];
      //cout << "px: " << px << endl;

      py = measurement_pack.raw_measurements_[1];
      //cout << "py: " << py << endl;

      // Initial state
      x_in << px, py, vx, vy;

      // Initialize
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

    }

    // modify timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  long long current_timestamp_ = measurement_pack.timestamp_;
  float elapsed_time = (current_timestamp_ - previous_timestamp_)/1000000.0;

  cout << "current_timestamp_  = " << current_timestamp_ << endl;
  //cout << "previous_timestamp_  = " << previous_timestamp_ << endl;
  //cout << "elapsed_time = " << elapsed_time << endl;

  float noise_ax = 9.0; // sigma_square ax
  float noise_ay = 9.0; // sigma_square ay

  MatrixXd Q_p;
  MatrixXd F_p;

  Q_p = MatrixXd(4,4);
  F_p = MatrixXd(4,4);

  // helping coefficients
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

  //cout << "pre-pred x_ = " << ekf_.x_ << endl;
  //cout << "pre-pred P_ = " << ekf_.P_ << endl;
  //cout << "pre-pred F_ = " << ekf_.F_ << endl;
  //cout << "pre-pred Q_ = " << ekf_.Q_ << endl;

  ekf_.Predict();

  cout << "PREDICTED x_ = " << ekf_.x_ << endl;
  cout << "PREDICTED P_ = " << ekf_.P_ << endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Nonlinear measurements require EKF update

    cout << "RADAR measurements " << measurement_pack.raw_measurements_ << endl;

    //Update Jacobian
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);


  } else {
    // Laser updates
    // Simple KF update mechanism can be used for linear measurements

    cout << "LIDAR measurements " << measurement_pack.raw_measurements_ << endl;

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // modify timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
