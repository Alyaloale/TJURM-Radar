#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;
 
int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src,undistorted;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是bgr格式 
    
    //********** 10 Hz        **********/
    ros::Rate loop_rate(30);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
        //图像去畸变
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        4.64815924e+03,0,2.06830014e+03,
        0,4.64989316e+03,1.22273070e+03,
        0,0,1.0000);  // 内参
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 
        -1.91884791e-01,
        1.13838789e-01,
        -2.70262719e-05,
        7.86028217e-04,
        3.29831618e-02);  // 畸变系数
        cv::undistort(src, undistorted, cameraMatrix, distCoeffs);

        //输出图像的信息
        ROS_INFO("=== Image Info ===");
        ROS_INFO("Size: %dx%d", src.cols, src.rows);
        ROS_INFO("Type: %s (CV_%dUC%d)", 
                typeToString(src.type()).c_str(),
                src.channels(),
                src.elemSize1() * 8);
        ROS_INFO("\n");
        // cv::resize(src, src, cv::Size(1080, 720));
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = undistorted;
#endif
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
    }
    return 0;
}
