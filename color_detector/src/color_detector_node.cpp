#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
using namespace std;
using namespace cv;

cv::Mat image;
cv::Mat HSVimage;
cv::Mat detect_screen_red;
cv::Mat image_copy;
cv::Mat thr;
vector<vector<Point>> contours;
int largest_contour_i(){
     int largest_area=0;
     int largest_contour_index=0;
     for( int i = 0; i< contours.size(); i++ ){
         double a=contourArea( contours[i],false);  
            if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;               
         }
     }
    return largest_contour_index;
};

void red_only(){
    vector<Vec4i> hierarchy;
    Moments m;

    cvtColor(image, HSVimage, cv::COLOR_BGR2HSV);// convert to  HSV
    inRange(HSVimage,cv::Scalar(0,50,50),cv::Scalar(15,255,255), detect_screen_red);// detect Red Colour
    erode(detect_screen_red, detect_screen_red, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)));
    findContours(detect_screen_red, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    image_copy = image.clone();

    drawContours(image_copy, contours, largest_contour_i(), Scalar(0, 255, 0), 2);
    threshold( detect_screen_red, thr, 100,255,THRESH_BINARY );
    if (largest_contour_i()==0)
        {m = moments(thr,true);}
    else
        {m = moments(contours[largest_contour_i()]);}
    Point p(m.m10/m.m00, m.m01/m.m00);
    circle(image_copy, p, 5, Scalar(255,255,255), -1);
    cout<< Mat(p);
    imshow("Image with center",image_copy);
    waitKey(1);
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = cv_ptr->image;
     red_only();
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