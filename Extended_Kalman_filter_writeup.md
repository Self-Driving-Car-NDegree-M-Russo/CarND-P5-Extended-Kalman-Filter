
## Extended Kalman Filter Project


The goal of this project is the implementation, in C++, of an Extended Kalman Filter (EKF) capable of tracking a vehicle using as input measurements from both a Radar and a Lidar sensor, both affected by noise.

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where three files where modified: [FusionEKF.cpp](./src/FusionEKF.cpp), [kalman_filter.cpp](./src/kalman_filter.cpp) and [tools.cpp](./src/tools.cpp). The other files have been left fundamentally unchanged.

The following sections of this writeup will provide details on the filter operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final section will show the results of the filter running against two different data sets.

[//]: # (Image References)

[image1]: ./pictures/Dataset_1_Final_Screenshot.png "Dataset 1 Final Results Screenshot"
[image2]: ./pictures/Dataset_2_Final_Screenshot.png "Dataset 2 Final Results Screenshot"

---
## Data Input

## Prediction

## Estimation

## Accuracy Evaluation

## Compiling the Code

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
Moreover, the screenshots show the final RMS error position and velocity, in both x and y directions. The requirement for the project is for these value to be below `[.11, .11, 0.52, 0.52]`, and, as it can be seen, this requirement is satisfied.

---
## Conclusions and Further Notes

