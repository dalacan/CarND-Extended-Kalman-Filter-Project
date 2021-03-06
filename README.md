# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

## Project outline
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Implementation Details

The predict and update functions of the Kalman Filter are implemented in the kalman_filter.cpp 
file. Specifically, the update function includes 3 update functions:

* `Update()`: Updates the state using the kalman filter equation
* `UpdateEKF()`: Update the state using the extended kalman filter equation
* `UpdateCommon()`: This function contains common kalman filter equations used by both 
the `Update()` and `UpdateEKF()` functions.

The FusionEKF.cpp file contains the usage implementation of the Kalman Filter. The code
* Initializes the Kalman filter depending on the measurement (Lidar or Radar).
* Calls the `KalmanFilter::Predict()`
* Calls the `KalmanFilter::Update()` if the measurement is Lidar
* Calls the `KalmanFilter::UpdateEKF()` if the measurement is Radar

Supporting functions have been implemented in the tools.cpp. These functions are:
* `CalculateRMSE`: Used to estimate the root mean square error.
* `CalculateJacobian`: Used to calculate the jacobian matrix for the given (px, py, vx, vy) matrix.

## Test Results

### Dataset 1
Below is the RMSE test results when the implementation was ran on dataset 1 from the Term 2 simulator.

| X | Y | VX | VY |
|:---:|:---:|:----:|:----:|
| 0.0974| 0.0855 | 0.4517 | 0.4404 |

### Dataset 2
Below is the RMSE test results when the implementation was ran on dataset 2 from the Term 2 simulator.

| X | Y | VX | VY |
|:---:|:---:|:----:|:----:|
| 0.0726| 0.0965 | 0.4219 | 0.4937 |

### Radar only
Below is the RMSE test results (on dataset 1) when only the radar sensor is used.

| X | Y | VX | VY |
|:---:|:---:|:----:|:----:|
| 0.2383| 0.3360| 0.5360 | 0.7172

As expected, when compared to dataset 1, the root mean square error is higher across all parameters. Implying a lower
certainty about the position and velocity.

### Lidar only
Below is the RMSE test results (on dataset 1) when only the laser sensor is used.

| X | Y | VX | VY |
|:---:|:---:|:----:|:----:|
| 0.1840| 0.1543| 0.6056 | 0.4862

As expected, when compared to dataset 1, the root mean square error is higher across all parameters. Implying a lower
certainty about the position and velocity. Additionally, when compared to the radar test, this results in a lower rmse. 
This is expected as the Lidar has a higher resolution than radar in when taking positioning measurements.