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
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
cv::Mat& process_image(cv::Mat& image) {
    // Make image gray
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    return image;
}
std::pair<std::vector<std::vector<cv::Point2f>>, int> secondary_detection(cv::Mat& image, std::vector<std::vector<cv::Point2f>> rejected){
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // 
}

    void detect_aruco(cv::Mat& image) {
    // Detect the markers in the image
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    // add a marker to dictionary

    // marker bits i
    image = process_image(image);
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);
    // Draw the markers
    // cv::aruco::drawDetectedMarkers(image, rejected, ids);
    // convert to rgb
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::aruco::drawDetectedMarkers(image, corners, ids);
    if(ids.size() > 0) {
        std::cout << "Detected " << ids.size() << " markers" << std::endl;
    }
    else{

    }
    cv::Mat copy = image.clone();
    cv::aruco::drawDetectedMarkers(copy, rejected, cv::noArray(), cv::Scalar(255, 0, 0));
    // Show the image
    cv::imshow("image", image);
    cv::waitKey(1);
    cv::imshow("rejected", copy);
    cv::waitKey(1);
}

cv::Mat image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert the image to OpenCV format
    try {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Detect the aruco markers
    detect_aruco(image);
}

std::string image_topic_name = "/camera/image_raw";
int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector_node");
    ros::NodeHandle nh;
    // Make a subscriber
    ros::Subscriber cam_sub = nh.subscribe(image_topic_name, 1, imageCallback);
    // Create a custom dictionary of 1 marker
    // markers of 6x6 bits
    // unsigned char data[7][7] = {{0, 0, 0, 0, 0, 0, 0},
    //     {0, 1, 0, 0, 0, 0, 0},
    //     {0, 1, 0, 0, 0, 0, 0},
    //     {0, 1, 0, 1, 1, 1, 0},
    //     {0, 1, 0, 1, 1, 1, 0},
    //     {0, 1, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0}};
    unsigned char data[6][6] = {{0, 0, 0, 0, 0, 0}, {0, 1, 1, 1, 0, 0}, {0, 1, 0, 1, 1, 0}, {0, 0, 1, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
    cv::Mat markerBits2(6, 6, CV_8UC1, data);
    cv::Mat markerCompressed = cv::aruco::Dictionary::getByteListFromBits(markerBits2);
    dictionary->bytesList.push_back(markerCompressed);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}