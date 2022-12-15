#include <ros/ros.h>
// include opencv
#include <opencv2/opencv.hpp>
// include cv_bridge
#include <cv_bridge/cv_bridge.h>
// include image_transport
#include <image_transport/image_transport.h>
// include sensor_msgs/Image.h
#include <sensor_msgs/Image.h>
// include aruco library
#include <opencv2/aruco.hpp>
// include custom messages
#include "aruco_detector.hpp"
#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

// empty 6 by 6 dictionary
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
Aruco::ArucoDetector detector(dictionary, parameters, "/camera/image_raw");
std::vector<std::vector<cv::Point2f> > corners;
std::vector<int> ids;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert the image to OpenCV format
    try {
        detector.image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Detect the aruco markers
    auto T = detector.detect_aruco();
    corners = T.first;
    ids = T.second;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector_node");
    ros::NodeHandle nh;
    // Make a subscriber
    ROS_INFO(detector.image_topic_name.c_str());
    ros::Subscriber cam_sub = nh.subscribe(detector.image_topic_name, 1, imageCallback);
    ros::Publisher aruco_detected_pub = nh.advertise<aruco_detector::aruco_detected>("aruco_detector/aruco_detected", 1);

    detector.dictionary = dictionary;
    dictionary->maxCorrectionBits = 3;
    // generate a 4x4 dictionary
    // write 5 markers to file from the 4x4 dictionar
    ros::Rate loop_rate(30);
    // Create message struct
    aruco_detector::aruco_detected msg;
    while (ros::ok()) {
        //   ROS_INFO("About to spin");
        ros::spinOnce();
        //   create message
        for (int i = 0; i < corners.size(); i++) {
            aruco_detector::aruco_message marker;
            marker.id = ids[i];
            for (int j = 0; j < corners[i].size(); j++) {
                geometry_msgs::Point point;
                point.x = corners[i][j].x;
                point.y = corners[i][j].y;
                marker.corners.push_back(point);
            }
            msg.detected_arucos.push_back(marker);
        }
        // publish message
        aruco_detected_pub.publish(msg);
        msg.detected_arucos.clear();
        loop_rate.sleep();
    }
    return 0;
}