#include <pose_estimator/pose_estimator.hpp>
#include <rosgraph_msgs/Log.h>
#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

Eigen::Vector3f pixel,coordi_cam_frame, world_frame,camera_pose;
Eigen::Matrix3f R,K,cam_height,camtoDrone;
float pix[3];
float center_x = 0.0, center_y = 0.0;
double cam_error_x = 0.0, cam_error_y = 0.0, cam_error_z = 0.0;
int count = 0;
bool flag = true;
//-----------------NOTE------------------//
// 1. Only one aruco should be detected  //
//                                       //
//---------------------------------------//
//-----------------NEED------------------//
// 1. Update cam_error_params from ground//
// 2. Be sure on camtoDrone matrix       //
//---------------------------------------//

void calc_pose()
{
    // Rotation matrix from camera to drone coordinate system
    camtoDrone <<  0.0f,-1.0f, 0.0f,
                  -1.0f, 0.0f, 0.0f,
                   0.0f, 0.0f,-1.0f;
    
    // Coordinates of center of aruco wrt camera frame
    coordi_cam_frame = (K.inverse())*cam_height*pixel;

    // world_frame = R_inv . coordi_cam_frame + camera_coordi_wrt_world_origin
    world_frame = (R.inverse())*(coordi_cam_frame);

    // world_frame = world_frame + camera_coordi_wrt_world_origin;
    world_frame = world_frame + (camtoDrone*camera_pose);

    // converting from drone frame to camera frame
    world_frame = camtoDrone*world_frame;

    ROS_INFO("Estimated ArUco Pose : x=%f y=%f z=%f",pixel[0],pixel[1],pixel[2]);
    return;
}

void center_callBack(const aruco_detector::aruco_detected &coords)
{
    center_x = 0.0, center_y = 0.0;
    for(int i=0; i<4; i++)
    {
        center_x += coords.detected_arucos[0].corners[i].x / 4.0;
        center_y += coords.detected_arucos[0].corners[i].y / 4.0;
        // sum_z = coords.detected_arucos[0].corners[i].x
    }
    pixel[0] = center_x;
    pixel[1] = center_y;
    pixel[2] = 1.0f;
    ROS_INFO("x=%f,y=%f,z=%f", coords.detected_arucos[0].corners[0].x, pixel[1],pixel[2]);
    return;
}

void camera_info_callBack(sensor_msgs::CameraInfo camera_params)
{
    if(flag) //Intrinsic Params do not change
    {
        for(int i =0; i<3; i++)
            for(int j=0; j<3; j++)
            {
                K(i,j) = camera_params.K[count];
                count++;
            }
    }
    count=0;
    for(int i =0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            R(i,j) = camera_params.R[count];
            count++;
        }
    flag=false;
    calc_pose();
    return;
}

void camera_pose_callBack(rosgraph_msgs::Log Sample) //(nav_msgs::Odometry drone_odom)
{
    camera_pose[0] = 0.0; //drone_odom.pose.pose.position.x - cam_error_x;
    camera_pose[1] = 0.0; //drone_odom.pose.pose.position.y - cam_error_y;
    camera_pose[2] = 0.72; //drone_odom.pose.pose.position.z - cam_error_z;

    // scaling_factor provided camera is nadir!
    for(int i =0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            if(i==j)
                cam_height(i,j) = camera_pose[2];
            else
                cam_height(i,j) = 0.0f;
        }
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pose_estimator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/aruco_detector/aruco_detected", 1, &center_callBack);
    ros::Subscriber sub_drone = nh.subscribe("/rosout", 1, &camera_pose_callBack); // /firefly/ground_truth/odometry
    ros::Subscriber sub_cam = nh.subscribe("/camera/camera_info", 1, &camera_info_callBack);
    ros::spin();
    return 0;
}