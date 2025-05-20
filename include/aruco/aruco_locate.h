#ifndef ARUCO_ARUCO_H
#define ARUCO_ARUCO_H_
#include "data_manager/param.h"
#include "data_manager/base.h"



void aruco_cereate();//生成ArUco标记
bool detectArucoMarkers(const cv::Mat& inputImage,
    std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f>>& markerCorners,
    bool drawMarkers ,std::vector<std::vector<cv::Point2f>>& rejectedCandidates);//检测ArUco标记
void aruco_detect();//检测ArUco标记

//获取一点在相机坐标系下方向向量
cv::Mat getDirectionVector(const cv::Point2f& point, const cv::Mat& intrinsicMatrix, const cv::Mat& distortionCoeffs);

//将方向向量转换为世界坐标系下的坐标
cv::Point3f direction2place(const cv::Mat &direction, cv::Point3f &origin, float targetX);
bool isPointInPolygon(const std::vector<cv::Point2f>& polygon, const cv::Point2f& P);
#endif