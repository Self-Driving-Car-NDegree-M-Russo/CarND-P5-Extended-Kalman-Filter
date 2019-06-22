
## Extended Kalman Filter Project


The goal of this project is the implementation, in C++, of an Extended Kalman Filter (EKF) capable of tracking a vehicle using as input measurements from both a Radar and a Lidar sensor, both affected by noise. The nonlinear nature of the problem is a consequence of the fact that the Radar sensor is capable of measuring position in terms of radial coordinates, while the Lidar measures directly cartesian ones. 

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where three files where modified: [FusionEKF.cpp](./src/FusionEKF.cpp), [kalman_filter.cpp](./src/kalman_filter.cpp) and [tools.cpp](./src/tools.cpp). The other files have been left fundamentally unchanged.
Furthermore, a [PDF document](./Docs/sensor-fusion-ekf-reference.pdf) has been provided as a support describing the equations used/implemented through the code. This was originally part of the Udacity training material provided, and so it is structured following the lessons schedule, but of course the equations still apply.

The following sections of this writeup will provide details on the filter operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final section will show the results of the filter running against two different data sets. 

[//]: # (Image References)

[image1]: ./pictures/Dataset_1_Final_Screenshot.png "Dataset 1 Final Results Screenshot"
[image2]: ./pictures/Dataset_2_Final_Screenshot.png "Dataset 2 Final Results Screenshot"

---
## Data Input

The data source for this EKF will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases).

This simulator makes use of data defined accordingly to a specific data format, an example of which is shown through 
the [data file](./data/obj_pose-laser-radar-synthetic-input.txt) provided with this repo, and then feeds them to the filter one line at a time. 

In the data file:

* Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).
* For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
* For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

The measurement values and timestamp are used subsequently in the EKF algorithm. Groundtruth is used for calculating root mean squared error.

The part of code that reads input from the simulator is in [main.cpp](./src/main.cpp), starting at line 61, where an instance  of the  ``MeasurementPackage`` class is created.

Then, for example, in lines 70-80 we can see the handling of Lidar measurements:

```sh
          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
```

The Radar case is defined analogously in lines 80-92.
The ground truth is read in lines 94-108:

```sh
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);
```

The measurements are processed through the command in line 111:

```sh
          // Call ProcessMeasurement(meas_package) for Kalman filter
          fusionEKF.ProcessMeasurement(meas_package);
```

## Initialization

The first thing that happens to the filter is to have its state initialized at the value of the first measurement ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 59-155).

The EKF implementation has a state composed by 4 variables: position and velocity of the tracked object, in 2D ([Ref. doc](./Docs/sensor-fusion-ekf-reference.pdf), pg. 2, eq. (18)). 
In case of initial reading, the position can be initialized to the current one of the tracked vehicle, while the velocity can be put to 0. Depending on wether the first measurement is a Radar or a Lidar one some trigonometric decomposition might be needed: this is handled in [FusionEKF.cpp](./src/FusionEKF.cpp), lines 107-137. For example, in the case of Radar we have (lines 111-118):

```sh
      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];

      px = rho * cos(theta);
      py = rho * sin(theta);

      // Initial state
      x_in << px, py, vx, vy;
```

NOTE: `vx, vy` are initialized = 0 on lines 76, 77.

Besides the state, the matrices can be intialized also. Specifically:

* The state matrix F has an expression that depnds on the elapsed time ([Ref. doc](./Docs/sensor-fusion-ekf-reference.pdf), pg. 3, eq. (21)). In case of first reading it can be initailized as a (4x4) identity matrix ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 95-99). Its actual value would be calculated and updated when processing next measurements.
* The process noise covariance matrix also has an expression depending on time ([Ref. doc](./Docs/sensor-fusion-ekf-reference.pdf), pg. 4, eq. (40)), but can be initialized to a (4x4) null matrix ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 89-93). Here too, the actual value will be calculated when new measurements become available.
* The estimation error covariance matrix can be initialized to a "big" value ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 101-105).

Other matices are actually defined in the EKF class contructor:

* The measurement noise covariance matrices for both the Laser and Radar have been actualy pre-provided as part of the Udacity starter code, and represent the charactirstics of the sensors ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 27-34).
* The measurement matrix for the Lidar case is a constant (2x4) ([Ref. doc](./Docs/sensor-fusion-ekf-reference.pdf), pg. 5, eq. (42)), and is introduced in [FusionEKF.cpp](./src/FusionEKF.cpp), lines 36-38.
* Finally, the measurement matrix for the Radar case will actually have to be calculated at every meadurement, given the nonlinear nature of the sensor. Initialization is just to a null (3x4) matrix ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 40-44).

With all the state and the matrices defined, the initialization happens by calling the `Init` method defined in [kalman_filter.cpp](./src/kalman_filter.cpp), lines 19-28. The actual parameters passed will depend on the nature of the first measurement; for example, in case of Radar we have ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 120,121):

```sh
      // Initialize
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);
```

while, for Lidar ([FusionEKF.cpp](./src/FusionEKF.cpp), lines 134,135):

```sh
      // Initialize
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);
```
      
## Prediction

After the initial reading has been used to initialize the filter, the subsequent ones can be normally processed, and the first step of the EKF algorithm is the prediction of the state, and the estimation error, at the time of the measurement, starting from the previous ones. This computation does not depend on the measurements, and so is the same in both Lidar and radar case.

The equations describing this step are documented in [Ref. doc](./Docs/sensor-fusion-ekf-reference.pdf) (pg. 2, eq. (11), (12)), and are implemented in the `Predict` method in [kalman_filter.cpp](./src/kalman_filter.cpp), lines 30-45:

```sh
      // Predicted State
      x_ = F_ * x_;

      MatrixXd Ft_ = F_.transpose();

      // Predicted P
      P_ = F_ * P_ * Ft_ + Q_;
```

It is worth noting that in writing these equations the process noise is assumed with mean = 0, and so there is no explicit `u` in the equation for state prediction.

Before the actual execution of this step, however, the state matrix F and the process covariance matrix need to be updated: their expression, in fact, depends on the actual elapsed time since the previous measurement.

The expression for F is documented in ([Ref. doc](./Docs/sensor-fusion-ekf-reference.pdf), pg. 3, eq. (21)), while the one for Q is at pg. 4 (eq. (40)). Their implementation can be found in [FusionEKF.cpp](./src/FusionEKF.cpp) (lines 151-191), and is imeediately followed by the call to the `Predict` method previously described (lines 193,194).

```sh
      // Predict
      ekf_.Predict();
```


## Estimation

## Accuracy Evaluation

## Compiling the Code

The code is intended to be compiled using CMake and Make. After having cloned this repo and taken care of the dependencies outlined in the repo [README](./README.md), you should just need to: 

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`

## Kalman Filter Tracking Results

Once the code is compiled it can be easily run through the command

```sh
./ExtendedKF
```

typed from the ``build`` folder.

The EKF code will look open a WebSocket server trying to connect to a data source, and that will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases).
Two different datasets can be simulated, and in both cases we should expect a good trajectory reconstruction. The following pictures show two screenshots from the simulator, for the final datapoint in both datasets. 

Final Screenshot - Dataset 1    |  Final Screenshot - Dataset 2
:-------------------------:|:-------------------------:
![alt text][image1] |  ![alt text][image2]

In both cases Lidar measurements are red circles, Radar measurements are blue circles and the green triangles represent the  trajectory estimation from the EKF.
Moreover, the screenshots show the final RMS error position and velocity, in both x and y directions. The requirement for the project is for these value to be below `[.11, .11, 0.52, 0.52]`, and, as it can be seen, it is satisfied in both cases.

---
## Conclusions and Further Notes

