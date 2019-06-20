
## Extended Kalman Filter Project


The goal of this project is the implementation, in C++, of an Extended Kalman Filter (EKF) capable of tracking a vehicle using as input measurements from both a Radar and a Lidar sensor, both affected by noise.

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where three files where modified: [FusionEKF.cpp](./src/FusionEKF.cpp), [kalman_filter.cpp](./src/kalman_filter.cpp) and [tools.cpp](./src/tools.cpp). The other files have been left fundamentally unchanged.

The following sections of this writeup will provide details on the filter operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final section will show the results of the filter running against two different data sets.

[//]: # (Image References)

[image1]: ./pictures/Dataset_1_Final_Screenshot.png "Dataset 1 Final Results Screenshot"
[image2]: ./pictures/Dataset_2_Final_Screenshot.png "Dataset 2 Final Results Screenshot"

---
## Data Collection

The data collection phase has probably been the most demanding part of this project. As it will be detailed in the next section, for the actual network design I decided to rely on something relatively consolidated; on the other hand, putting together an effective data set, or better a combination of datasets, took several different attemps and quite a bit of trial-and-error.

Finally, the best results have been obtained making use of 3 data sets:

* A "clean" driving of the track, operated by me;
* A "reverse" driving of the track, in which, after a U-turn at the very beginning I've driven the car in the opposite of the normal direction;
* A data set provided as reference directly by Udacity.

Despite the relative simplicity of these scenarios, their combination succeeded in producing a valid round of the vehicle along the track. Please note that, given their size, the datasets have NOT been attached to the current version of the project.

The datasets have been further enriched as decribed in the `readx3()` helper function provided as part of [model.py](./model.py) script (lines 31-65). In particular, for all the data sets the images from all the cameras are used: the recorder steering angle is used as a label for the image coming from the middle camera, while the one to be used the left/right camera is obtained from the recorded angle +/- a fixed (configurable) correction value. Furthermore every image has been vertically flipped and the relative steering angle changed in sign, so to emulate a run of the track in reverse.

The three datasets are firstly parsed making use of the `driving_log.csv` logfile that gets generated with every run ([model.py](./model.py), lines 160-183). Lines in the logfile are ordered cronologically, and every one of them looks like this:


|../My-data/IMG/center.jpg	|../My-data/IMG/left.jpg |../My-data/IMG/right.jpg	|-1.36	|0	|0	|9.35 |
|:------------:|:----------:|:-------------:|:------------:|:------------:|:-----------:|:-------------:|

It contains the identifiers for the image files coming from the 3 cameras (first 3 fields) as well as the steering angle at the moment of the capture (fourth field) and the speed (last field). On top of these information I decided to append a "flag", i.e. an identifier for each dataset (0/1/2), that will get used when accessing the actual images.

The content of the datasets is split in Train/Validation in an 80/20 percentage using the `train_test_split` function imported from `sklearn` ([model.py](./model.py), lines 186-188)

The three datasets are then collected together using the `generator()` helper function ([model.py](./model.py), line 69-143). A _generator_ is a [specific kind of function](https://wiki.python.org/moin/Generators) allowed by the Python language, that can be declared as iterator, i.e. used in a for loop. They are characterized by the usage of the `yield` keyword, and can be used when a piece of code has to access elements from a list without loading the full list in memory. This is helpful in case of lists with a heavy footprint, like the one that we are considering for this code.
In this case the `generator()` helper will return, every time, a _shuffled_ batch of 32 images/labels taken from the global dataset.

## Model Design and Training

The design of the Network implemented here is based on the Nvidia solution presented in the Udacity class. Including cropping and normalizing steps the CNN layers can be described as it follows ([model.py](./model.py), lines 213-233):


| Layer         		|     Description	        					|  
|:---------------------:|:---------------------------------------------:|
|Input    | Original Image | 
|Pre-processing    | Cropping | 
|Pre-processing    | Normalize data | 
|Convolution    | 5x5, Depth 24, Padding 2  | 
|Convolution    | 5x5, Depth 36, Padding 2  | 
|Convolution    | 5x5, Depth 48, Padding 2  | 
|Convolution    | 3x3, Depth 64  | 
|Convolution    | 3x3, Depth 64  | 
|Flatten    | Transition from convolutional to dense  | 
|Dense    | Depth 100  | 
|Dense    | Depth 50  | 
|Dense    | Depth 10  | 
|Dense    | Depth 1  (Final classifier)| 


Furthermore:

* All the activation functions are RELUs;
* The used optimizer is Adam ([model.py](./model.py), line 236);
* After a couple of tries, I decided to settle for 2 Epochs, with a final accuracy for the model of 98.13%. The total training time was about 40 minutes on a cloud-hosted GPU. 

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

