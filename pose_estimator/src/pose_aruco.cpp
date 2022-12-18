#include <pose_estimator/pose_estimator.hpp>

float center_x = 0.0, center_y = 0.0;
double cam_error_x = 0.0, cam_error_y = 0.0, cam_error_z = 0.0;
int count = 0; 
bool flag = true;

std::vector<std::vector<cv::Point2f>> c;
cv::Mat cameraMatrix, distCoeffs;
Eigen::Matrix3f R,K;
std::vector<cv::Vec3d> rvecs, tvecs;
cv::Point2f pt;

void calc_pose()
{
    std::cout << "Before the algo \n";
    cv::aruco::estimatePoseSingleMarkers(c, 0.5, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout << "-----" <<std::endl;
    for(auto i : rvecs)
        std::cout << i <<std::endl;
    std::cout << "----" <<std::endl;
    std::cout << "After the algo\n";
}
void center_callBack(const aruco_detector::aruco_detected::ConstPtr& coords)
{
    std::cout << "Gaining the center coorids\n";
    if(coords->detected_arucos.size() == 0)
        ROS_WARN("ArUco Not Detected. Using previous pose");
    else
    {
        // Way One
        // for(int i=0; i<coords.detected_arucos.size(); i++)
        // {
        //     for(int j=0; j<4; j++)
        //     {
        //         pt.x = coords.detected_arucos[i].corners[j].x;
        //         pt.y = coords.detected_arucos[i].corners[j].y;
        //         c[i].push_back(pt);
        //     }
        // }
        // Way Two
        std::cout << "error1 to be checked\n";
        // c[0].push_back(cv::Point2f(coords->detected_arucos[0].corners[0].x,coords->detected_arucos[0].corners[0].y));
        // c[0].push_back(coords.detected_arucos[0].corners[1]);
        // c[0].push_back(coords.detected_arucos[0].corners[2]);
        // c[0].push_back(coords.detected_arucos[0].corners[3]);
        // Way Three
        // top left corner 
        std::cout << "c[0][0]= " << (coords->detected_arucos[0].corners[0].x);
        std::cout << "c[0][0]= " << (coords->detected_arucos[0].corners[0].y);
        c[0][0].x = (coords->detected_arucos[0].corners[0].x);
        c[0][0].y = (coords->detected_arucos[0].corners[0].y);

        std::cout << "error1 not an error...yay!\n";
        // // // top right corner
        // std::cout << "error2 to be checked \n";
        // c[0][1].x = (coords->detected_arucos[0].corners[1].x);
        // c[0][1].y = (coords->detected_arucos[0].corners[1].y);
        // // // bottom right corner
        // std::cout << "error3 to be checked\n";
        // c[0][2].x = (coords->detected_arucos[0].corners[2].x);
        // c[0][2].y = (coords->detected_arucos[0].corners[2].y);
        // // //bottom left corner
        // std::cout << "error4 to be checked\n";
        // c[0][3].x = (coords->detected_arucos[0].corners[3].x);
        // c[0][3].y = (coords->detected_arucos[0].corners[3].y);

        std::cout << "Before calling calc_pose\n";
        // calc_pose();
        std::cout << "After calling calc_pose\n";
    }
    return;
}

void camera_info_callBack(const sensor_msgs::CameraInfo camera_params)
{
    // std::cout << "Just entered camera_info_callback\n";
    float CAM[9] = {963.304385, 0.000000, 637.039646, 0.000000, 965.910111, 479.796409, 0.000000, 0.000000, 1.000000};
    float DIS[5] = {-0.411342, 0.158145, 0.000504, 0.000315, 0.000000};
    cameraMatrix = cv::Mat(1,9,CV_32F,CAM);
    distCoeffs = cv::Mat(1,5,CV_32F,DIS);
    // std::cout << cameraMatrix <<"\n"<< distCoeffs << std::endl;
    return;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pose_new");
    ros::NodeHandle nh;
    // std::cout << "Lets do this bois\n";
    ros::Subscriber sub_cam = nh.subscribe("/camera/camera_info", 1, &camera_info_callBack);
    ros::Subscriber sub = nh.subscribe("/aruco_detector/aruco_detected", 1, &center_callBack);
    ros::spin();
    return 0;
}