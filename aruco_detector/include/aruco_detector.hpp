#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace Aruco {
class ArucoDetector {
  public:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Mat image;
    std::string image_topic_name;

    cv::Rect get_expanded_rectangle(std::vector<cv::Point2f> corners, int extra_height, int original_cols, int original_rows);
    std::pair<std::vector<std::vector<cv::Point2f>>, int> secondary_detection(std::vector<std::vector<cv::Point2f>> rejected);
    void detect_aruco();

    ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dictionary_, cv::Ptr<cv::aruco::DetectorParameters> parameters_, std::string image_topic_name_);
    ~ArucoDetector();
};
}  // namespace Aruco