#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>

#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

#include "pose_estimator/pose_estimator.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rosgraph_msgs/Log.h>

namespace Pose {
PoseEstimator::PoseEstimator(ros::NodeHandle nh_, std::string aruco_sub_topic, std::string drone_sub_topic, std::string cam_sub_topic, std::string pub_topic) {
    nh = nh_;
    sub = nh_.subscribe(aruco_sub_topic, 1, &PoseEstimator::center_callBack, this);
    sub_drone = nh_.subscribe(drone_sub_topic, 1, &PoseEstimator::camera_pose_callBack, this);
    sub_cam = nh_.subscribe(cam_sub_topic, 1, &PoseEstimator::camera_info_callBack, this);
    pub = nh_.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);

    return;
}
void PoseEstimator::calc_pose() {
    camtoDrone << 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f;
    // Coordinates of center of aruco wrt camera frame
    coordi_cam_frame = (K.inverse()) * pixel;
    // check all values of coordi_cam_frame
    coordi_cam_frame = cam_height * coordi_cam_frame;
    // world_frame = R_inv . coordi_cam_frame + camera_coordi_wrt_world_origin
    world_frame = (coordi_cam_frame);
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = world_frame[0];
    pose.pose.position.y = world_frame[1];
    pose.pose.position.z = world_frame[2];
    pub.publish(pose);
    // world_frame = world_frame + camera_coordi_wrt_world_origin;
    // world_frame = world_frame + (camtoDrone * camera_pose);

    // converting from drone frame to camera frame
    // world_frame = camtoDrone * world_frame;

    return;
}
void PoseEstimator::center_callBack(const aruco_detector::aruco_detected::ConstPtr& coords) {
    if (coords->detected_arucos.size() != 0) {
        center_x = 0.0, center_y = 0.0;
        for (int i = 0; i < 4; i++) {
            center_x += coords->detected_arucos[0].corners[i].x / 4.0;
            center_y += coords->detected_arucos[0].corners[i].y / 4.0;
            // sum_z = coords.detected_arucos[0].corners[i].x
        }
        pixel[0] = center_x;
        pixel[1] = center_y;
        pixel[2] = 1.0f;

        // ROS_INFO("x=%f,y=%f,z=%f", coords.detected_arucos[0].corners[0].x, pixel[1],pixel[2]);
    }
    // get average side length

    return;
}
void PoseEstimator::camera_info_callBack(const sensor_msgs::CameraInfo::ConstPtr& camera_params) {
    if (flag)  // Intrinsic Params do not change
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                K(i, j) = camera_params->K[count];
                // if ((i == 0 && j == 0) || (i == 1 && j == 1))
                //     K(i, j) = K(i, j) * 0.01;
                count++;
            }
        // get inverse
    }

    count = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            R(i, j) = camera_params->R[count];
            count++;
        }
    flag = false;
    calc_pose();
    return;
}
void PoseEstimator::camera_pose_callBack(const rosgraph_msgs::Log Sample) {
    camera_pose[0] = 0.0;    // drone_odom.pose.pose.position.x - cam_error_x;
    camera_pose[1] = 0.0;    // drone_odom.pose.pose.position.y - cam_error_y;
    camera_pose[2] = 150.0;  // drone_odom.pose.pose.position.z - cam_error_z;

    // scaling_factor provided camera is nadir!
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            if (i == j)
                cam_height(i, j) = camera_pose[2];
            else
                cam_height(i, j) = 0.0f;
        }
    return;
}
}  // namespace Pose