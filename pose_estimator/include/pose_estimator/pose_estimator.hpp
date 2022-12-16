// # pragma once

#include <bits/stdc++.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>

#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

#include <geometry_msgs/PoseStamped.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
namespace Pose {
class PoseEstimator {
  public:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub_drone;  // /firefly/ground_truth/odometry
    ros::Subscriber sub_cam;
    ros::Publisher pub;

    Eigen::Vector3f pixel;
    Eigen::Vector3f coordi_cam_frame;
    Eigen::Vector3f world_frame;
    Eigen::Vector3f camera_pose;

    Eigen::Matrix3f R;
    Eigen::Matrix3f K;
    Eigen::Matrix3f cam_height;
    Eigen::Matrix3f camtoDrone;

    float pix[3];
    float center_x = 0.0;
    float center_y = 0.0;
    double cam_error_x = 0.0;
    double cam_error_y = 0.0;
    double cam_error_z = 0.0;
    int count = 0;
    bool flag = true;

    geometry_msgs::PoseStamped pose;

    void calc_pose();
    void camera_info_callBack(const sensor_msgs::CameraInfo::ConstPtr& camera_params);
    void center_callBack(const aruco_detector::aruco_detected::ConstPtr& msg);
    void camera_pose_callBack(const rosgraph_msgs::Log Sample);  //(nav_msgs::Odometry drone_odom)
    PoseEstimator(ros::NodeHandle nh_, std::string aruco_sub_topic, std::string drone_sub_topic, std::string cam_sub_topic, std::string pub_topic);
    ~PoseEstimator(){};
};
}  // namespace Pose