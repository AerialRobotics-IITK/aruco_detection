#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "aruco_detector.hpp"

namespace Aruco {

ArucoDetector::ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dictionary_, cv::Ptr<cv::aruco::DetectorParameters> parameters_, std::string image_topic_name_) {
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    parameters = cv::aruco::DetectorParameters::create();
    image_topic_name = image_topic_name_;
}

ArucoDetector::~ArucoDetector() {
}

cv::Rect ArucoDetector::get_expanded_rectangle(std::vector<cv::Point2f> corners, int extra_height, int original_cols, int original_rows) {
    auto rectangle = cv::boundingRect(corners);

    rectangle.x = std::max(0, rectangle.x - extra_height);
    rectangle.y -= std::max(0, rectangle.y - extra_height);
    rectangle.width = std::min(original_cols - rectangle.x, rectangle.width + 2 * extra_height);
    rectangle.height = std::min(original_rows - rectangle.y, rectangle.height + 2 * extra_height);

    return rectangle;
}

std::pair<std::vector<std::vector<cv::Point2f>>, int> ArucoDetector::secondary_detection(std::vector<std::vector<cv::Point2f>> rejected) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // remove the rejected ones that are too small
    rejected.erase(std::remove_if(rejected.begin(), rejected.end(), [](std::vector<cv::Point2f> i) { return cv::contourArea(i) < 700; }), rejected.end());
    // order in descending order of area
    std::sort(rejected.begin(), rejected.end(), [](std::vector<cv::Point2f> i, std::vector<cv::Point2f> j) { return cv::contourArea(i) > cv::contourArea(j); });

    for (auto i : rejected) {
        for (int height = 10; height <= 120; height = height + 10) {
            auto rectangle = get_expanded_rectangle(i, height, image.cols, image.rows);

            cv::Mat subimage = image(rectangle);
            cv::cvtColor(subimage, subimage, cv::COLOR_BGR2GRAY);
            cv::waitKey(1);
            cv::threshold(subimage, subimage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            cv::aruco::detectMarkers(subimage, dictionary, corners, ids, parameters);
            if (ids.size() > 0) {
                // correct the corners
                for (auto& j : corners) {
                    for (auto& k : j) {
                        k.x += rectangle.x;
                        k.y += rectangle.y;
                    }
                }
                return std::make_pair(corners, ids[0]);
            }
        }
    }
    return std::make_pair(corners, -1);
}

void ArucoDetector::detect_aruco() {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);

    if (ids.size() > 0) {
        // do nothing
    } else {
        auto corners_ids = secondary_detection(rejected);
        if (corners_ids.second != -1) {
            corners = corners_ids.first;
            ids.push_back(corners_ids.second);
            ROS_ERROR("Secondary detection");
        }
    }
    cv::Mat copy = image.clone();
    if (ids.size() > 0 && corners.size() != ids.size()) {
        // truncate corners to match ids
        corners.resize(ids.size());
    }
    cv::aruco::drawDetectedMarkers(image, corners, ids);
    cv::aruco::drawDetectedMarkers(copy, rejected, cv::noArray(), cv::Scalar(255, 0, 0));
    cv::imshow("image", image);
    cv::waitKey(1);
    cv::imshow("rejected", copy);
    cv::waitKey(1);
}
}  // namespace Aruco