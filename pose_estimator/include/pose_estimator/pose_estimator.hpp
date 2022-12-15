// # pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <bits/stdc++.h>

#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>