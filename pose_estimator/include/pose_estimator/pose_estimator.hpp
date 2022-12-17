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
#include <pose_estimator/pose_message.h>
#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

#include <geometry_msgs/PointStamped.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
namespace Pose {
class PoseEstimator {
  public:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub_drone;  // /firefly/ground_truth/odometry
    ros::Subscriber sub_cam;
    ros::Subscriber sub_color;
    ros::Publisher pose_aruco_pub;
    ros::Publisher pose_color_pub;
    ros::Subscriber mavros_sub;

    Eigen::Vector3f pixel;
    Eigen::Vector3f cam_frame;
    Eigen::Vector3f camera_pose;
    Eigen::Vector3f world_frame;
    Eigen::Vector3f rolling_avg_wf;

    Eigen::Matrix3f R;
    Eigen::Matrix3f K;
    Eigen::Matrix3f cam_height;
    Eigen::Matrix3f camtoDrone;
    float drone_height;

    Eigen::Matrix3f cam_rot_mat;
    Eigen::Matrix3f image_rot_mat_wrt_cam;
    Eigen::Matrix3f cam_rot_mat_inversed;
    Eigen::Matrix3f image_rot_mat_wrt_cam_inversed;
    float center_x = 0.0;
    float center_y = 0.0;
    double cam_error_x = 0.0;
    double cam_error_y = 0.0;
    double cam_error_z = 0.0;
    int count = 0;
    bool flag = true;
    float rolling_avg_count;

    void calc_pose(bool from_aruco = false, bool from_color = false);
    void camera_info_callBack(const sensor_msgs::CameraInfo::ConstPtr& camera_params);
    void center_callBack(const aruco_detector::aruco_detected::ConstPtr& msg);
    // NEW
    void color_pose_callBack(const geometry_msgs::Point& color_center);
    void camera_pose_callBack(const rosgraph_msgs::Log Sample);
    void drone_orientation_callBack(const sensor_msgs::Imu::ConstPtr& drone_odom);
    void get_camera_rotation_matrix();
    void get_image_rotation_matrix_wrt_cam();
    float z_rotation_correction();
    void compute_pose_in_world_frame();
    void publish_aruco_pose(bool detected = true);
    //NEW
    void publish_color_pose(bool detected = true);
    void compute_rolling_avg();
    void set_msg_to_not_detected(geometry_msgs::PoseStamped& pose);
    PoseEstimator(ros::NodeHandle nh_,
        std::string aruco_sub_topic,
        std::string color_sub_topic,
        std::string drone_sub_topic,
        std::string cam_sub_topic,
        std::string pub_topic,
        float rolling_avg_count_);
    ~PoseEstimator(){};
};
}  // namespace Pose