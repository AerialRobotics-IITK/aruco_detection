#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

cv::Mat image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = cv_ptr->image;
    cv::imshow("image", image);
    cv::waitKey(1);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "color_detector_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber cam_sub = it.subscribe("/camera/image_raw", 1, imageCallback);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}