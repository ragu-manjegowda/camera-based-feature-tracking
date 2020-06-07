# Camera based 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea is to the feature tracking part and test various detector / descriptor 
combinations to see which ones perform best. This mid-term project consists of 
four parts:

* First, loading images, setting up data structures and putting everything into a ring 
buffer to optimize memory load. 
* Then, integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and 
compare them with regard to number of keypoints and speed. 
* In the next part, descriptor extraction and matching using brute force and also the 
FLANN approach. 
* In the last part, once the code framework is complete, test various algorithms in 
different combinations and compare them with regard to some performance measures. 


## Dependencies for Running Locally

### Consider using docker image,

```
$ docker pull ragumanjegowda/docker
```

### For native/host compilation (Note: I have not tested this!)

* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

```
$ mkdir build && cd build
$ cmake -G Ninja .. && ninja
$ ./2D_feature_tracking
```
