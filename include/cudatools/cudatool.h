#ifndef CUDA_CUDATOOL_H
#define CUDA_CUDATOOL_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "cudatools/cuda.cuh"
#include <Eigen/Dense>


// CUDA错误检查宏
#define CHECK_CUDA(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error: " << cudaGetErrorString(err) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

void depth_to_pointcloud( std::vector<float>* &point,cv::Mat& depth,
    float fx, float fy, float cx, float cy, float scale);//深度图转点云


void point_transform(
    std::vector<float>* &points,
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& T);//将相机坐标系下的点进行坐标变换


void project_pointcloud_to_depth(
    std::vector<float>* &points, 
    std::vector<float>* &depth,
    float fx, float fy, float cx, float cy,
    int width, int height);//将点云投影到深度图
#endif
