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

#include "aruco_detector.hpp"

// empty 6 by 6 dictionary
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(0, 6);
// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
Aruco::ArucoDetector detector(dictionary, parameters, "/camera/image_raw");

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert the image to OpenCV format
    try {
        detector.image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Detect the aruco markers
    detector.detect_aruco();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector_node");
    ros::NodeHandle nh;
    // Make a subscriber
    ROS_INFO(detector.image_topic_name.c_str());
    ros::Subscriber cam_sub = nh.subscribe(detector.image_topic_name, 1, imageCallback);
    // Create a custom dictionary of 1 marker
    // markers of 6x6 bits
    // unsigned char data[7][7] = {{0, 0, 0, 0, 0, 0, 0},
    //     {0, 1, 0, 0, 0, 0, 0},
    //     {0, 1, 0, 0, 0, 0, 0},
    //     {0, 1, 0, 1, 1, 1, 0},
    //     {0, 1, 0, 1, 1, 1, 0},
    //     {0, 1, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0}};
    // unsigned char data[4][4] = {{0, 0, 0, 0, 0, 0}, {0, 1, 1, 1, 0, 0}, {0, 1, 0, 1, 1, 0}, {0, 0, 1, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
    unsigned char data[4][4] = {{1, 0, 1, 1}, {0, 0, 1, 1}, {1, 0, 0, 1}, {1, 1, 1, 0}};
    cv::Mat markerBits2(4, 4, CV_8UC1, data);
    cv::Mat markerCompressed = cv::aruco::Dictionary::getByteListFromBits(markerBits2);
    dictionary->bytesList.push_back(markerCompressed);
    unsigned char data_[4][4] = {{1, 1, 1, 0}, {1, 0, 1, 1}, {0, 1, 0, 0}, {1, 0, 0, 0}};
    cv::Mat markerBits(4, 4, CV_8UC1, data_);
    markerCompressed = cv::aruco::Dictionary::getByteListFromBits(markerBits);
    dictionary->bytesList.push_back(markerCompressed);
    // show all parameters
    std::cout << "adaptiveThreshWinSizeMin: " << parameters->adaptiveThreshWinSizeMin << std::endl;
    std::cout << "adaptiveThreshWinSizeMax: " << parameters->adaptiveThreshWinSizeMax << std::endl;
    std::cout << "adaptiveThreshWinSizeStep: " << parameters->adaptiveThreshWinSizeStep << std::endl;
    std::cout << "adaptiveThreshConstant: " << parameters->adaptiveThreshConstant << std::endl;
    std::cout << "minMarkerPerimeterRate: " << parameters->minMarkerPerimeterRate << std::endl;
    std::cout << "maxMarkerPerimeterRate: " << parameters->maxMarkerPerimeterRate << std::endl;
    std::cout << "polygonalApproxAccuracyRate: " << parameters->polygonalApproxAccuracyRate << std::endl;
    std::cout << "minCornerDistanceRate: " << parameters->minCornerDistanceRate << std::endl;
    std::cout << "minDistanceToBorder: " << parameters->minDistanceToBorder << std::endl;
    std::cout << "minMarkerDistanceRate: " << parameters->minMarkerDistanceRate << std::endl;
    std::cout << "cornerRefinementWinSize: " << parameters->cornerRefinementWinSize << std::endl;
    std::cout << "cornerRefinementMaxIterations: " << parameters->cornerRefinementMaxIterations << std::endl;
    std::cout << "cornerRefinementMinAccuracy: " << parameters->cornerRefinementMinAccuracy << std::endl;
    std::cout << "markerBorderBits: " << parameters->markerBorderBits << std::endl;
    std::cout << "perspectiveRemovePixelPerCell: " << parameters->perspectiveRemovePixelPerCell << std::endl;
    std::cout << "perspectiveRemoveIgnoredMarginPerCell: " << parameters->perspectiveRemoveIgnoredMarginPerCell << std::endl;
    std::cout << "maxErroneousBitsInBorderRate: " << parameters->maxErroneousBitsInBorderRate << std::endl;
    std::cout << "minOtsuStdDev: " << parameters->minOtsuStdDev << std::endl;
    std::cout << "errorCorrectionRate: " << parameters->errorCorrectionRate << std::endl;
    // check all fields in the dictionary
    dictionary->markerSize = 4;
    dictionary->maxCorrectionBits = 3;
    std::cout << "dictionary->bytesList.size(): " << dictionary->bytesList.size() << std::endl;
    std::cout << "dictionary->markerSize: " << dictionary->markerSize << std::endl;
    std::cout << "dictionary->maxCorrectionBits: " << dictionary->maxCorrectionBits << std::endl;
    //   dictionary = cv::aruco::getPredefinedDictionary(cv::DICT_4X4_50);
    detector.dictionary = dictionary;
    // generate a 4x4 dictionary
    // write 5 markers to file from the 4x4 dictionar
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        //   ROS_INFO("About to spin");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}