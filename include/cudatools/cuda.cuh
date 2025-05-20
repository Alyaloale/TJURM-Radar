#ifndef CUDA_CUDA_CH
#define CUDA_CUDA_CH


#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include <algorithm>
#include <cmath>
#include <atomic>


__global__ void depth_to_pointcloud_kernel(float* points, const float* depth,
    int width, int height,
    float fx, float fy, float cx, float cy, float scale);//CUDA核函数：深度图转点云
__global__ void point_transform_kernel(float* points, size_t num_points,
    const float* R, const float* T);//CUDA核函数：坐标系转换
__global__ void project_pointcloud_to_depth_kernel(
    const float* __restrict__ points, 
    size_t num_points,
    float* __restrict__ depth,
    int width, int height,
    float fx, float fy, float cx, float cy,
    float min_z, float max_z);//CUDA核函数：将点云投影到深度图


#endif