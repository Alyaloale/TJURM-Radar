#include "aruco/aruco_locate.h"



void aruco_cereate()
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    for(int i=0;i<4;i++)
    {
        cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
        cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/aruco/"+std::to_string(i)+".jpg", markerImage);
    }
}


//  检测 ArUco 标记的函数
bool detectArucoMarkers(const cv::Mat& inputImage,
    std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f>>& markerCorners,
    bool drawMarkers ,std::vector<std::vector<cv::Point2f>>& rejectedCandidates)
{
    cv::Mat imageCopy;
    inputImage.copyTo(imageCopy);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(Data::dictionaryName);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::detectMarkers(imageCopy, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
    if (drawMarkers) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    }
    return !markerIds.empty();
}

void aruco_detect() {
    while(true)
    {
        memcpy(Data::camera[0]->image, Data::camera[0]->image_buffer, Data::camera[0]->height * Data::camera[0]->width * 3);
        cv::Mat image = cv::Mat(Data::camera[0]->height, Data::camera[0]->width, CV_8UC3, Data::camera[0]->image);
        if(!image.empty())
        {
            bool IsdetectAruco;
            Data::markerIds.clear();
            Data::markerCorners.clear();
            IsdetectAruco = detectArucoMarkers(image, Data::markerIds, Data::markerCorners, true, Data::rejectedCandidates);
            //画出被拒绝的候选框
            //cv::aruco::drawDetectedMarkers(image, Data::rejectedCandidates, std::vector<int>());
            if(IsdetectAruco)
            {
                //画出检测到的aruco
                cv::aruco::drawDetectedMarkers(image, Data::markerCorners, Data::markerIds);
                //画出被拒绝的候选点
                //计算标记在三维空间中的位置和姿态。
                // cv::Mat rvec, tvec;
                // cv::aruco::estimatePoseSingleMarkers(Data::markerCorners, 0.07, Data::camera[0]->intrinsic_matrix, Data::camera[0]->distortion_coeffs, rvec, tvec);
                // for(int i = 0; i < Data::markerIds.size(); i++)
                // {
                //     cv::aruco::drawAxis(image, Data::camera[0]->intrinsic_matrix, Data::camera[0]->distortion_coeffs, rvec.at<cv::Vec3d>(i), tvec.at<cv::Vec3d>(i), 0.1);
                //     std::cout << "Marker ID: " << Data::markerIds[i] << std::endl;
                //     std::cout << "Rotation Vector: " << rvec.at<cv::Vec3d>(i) << std::endl;
                //     std::cout << "Translation Vector: " << tvec.at<cv::Vec3d>(i) << std::endl;
                // }

                cv::Point2f center;
                for (size_t i = 0; i < Data::markerCorners.size(); i++) {
                    // 计算每个标记的中心点
                    center.x = (Data::markerCorners[i][0].x + Data::markerCorners[i][2].x) / 2;
                    center.y = (Data::markerCorners[i][0].y + Data::markerCorners[i][2].y) / 2;
                    // 在图像上绘制中心点
                    cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);
                    cv::Mat direction = getDirectionVector(center, Data::camera[0]->intrinsic_matrix, Data::camera[0]->distortion_coeffs);
                    cv::Point3f origin = cv::Point3f(0, 0, 0);
                    cv::Point3f intersection = direction2place(direction, origin, 0.8);
                    double depth = cv::norm(intersection);
                    std::cout<<"intersection: "<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;
                    std::cout<<"depth: "<<depth<<std::endl;
                }
            }

            cv::imshow("Aruco Detection", image);
            cv::waitKey(1);
        }
    }
}