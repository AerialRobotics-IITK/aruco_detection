# aruco_detection
## aruco detector 
This package detects any aruco markers(registered in the dictionary) present in the frame of the OCam.

## pose estimator
This package calculates the pose of the detected aruco marker with respect to the world frame.

## Installation
```
catkin build
roslaunch <launch-file>.launch #launch-file = aruco_node / color_node / ocam_ros / generate_pose

```

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin init 
git clone https://github.com/AerialRobotics-IITK/aruco_detection.git .
```

## Troubleshooting
* 
    ```
    fatal error: aruco_detector/aruco_detected.h: No such file or directory
    1 | #include "aruco_detector/aruco_detected.h"
    ```
    On the first run of ``catkin build`` or ``catkin_make``, it compiles the .msg files to .h files. Run ``catkin build`` or ``catkin_make`` again to get rid of the error

* 
    ```
    fatal error: libv4l2.h: No such file or directory
    27 | #include <libv4l2.h>
    
    ```
    You will need to install v4l2 libraries first. First run, ``sudo apt-get update``. Then, run ``sudo apt-get install libv4l-dev`` and ``sudo apt-get -y install v4l-utils``
 

## Issues' Changelog
* 1/8/2023 - added Troubleshooting and Installation in README.md
