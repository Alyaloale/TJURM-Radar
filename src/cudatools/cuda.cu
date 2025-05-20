#include <iostream>
#include <opencv2/opencv.hpp>
#include "cudatools/cuda.cuh"

// CUDA核函数：深度图转点云
__global__ void depth_to_pointcloud_kernel(float* points, const float* depth,
    int width, int height,
    float fx, float fy, float cx, float cy, float scale) {
    int u = blockIdx.x * blockDim.x + threadIdx.x; // 像素列索引
    int v = blockIdx.y * blockDim.y + threadIdx.y; // 像素行索引

    if (u < width && v < height) {
        float d = depth[v * width + u] / scale; // 深度值
        if (d <= 0 || std::isinf(d)) return; // 忽略无效深度值
            // 转换为三维点
            float x = (u - cx) * d / fx;
            float y = (v - cy) * d / fy;
            float z = d;

            // 将结果写入 points
            int idx = (v * width + u) * 3;
            points[idx] = x;
            points[idx + 1] = y;
            points[idx + 2] = z;
    }
}

//CUDA核函数：坐标系变换
__global__ void point_transform_kernel(float* points, size_t num_points,
    const float* R, const float* T) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x; // 当前点的索引

    if (idx < num_points) {
        // 获取当前点
        float x = points[idx * 3];
        float y = points[idx * 3 + 1];
        float z = points[idx * 3 + 2];

        // 应用旋转和平移
        float x_new = R[0] * x + R[1] * y + R[2] * z + T[0];
        float y_new = R[3] * x + R[4] * y + R[5] * z + T[1];
        float z_new = R[6] * x + R[7] * y + R[8] * z + T[2];

        // 更新点云
        points[idx * 3] = x_new;
        points[idx * 3 + 1] = y_new;
        points[idx * 3 + 2] = z_new;
    }
}

// CUDA核函数：将点云投影到深度图
__global__ void project_pointcloud_to_depth_kernel(
    const float* __restrict__ points, 
    size_t num_points,
    float* __restrict__ depth,
    int width, int height,
    float fx, float fy, float cx, float cy,
    float min_z, float max_z)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    // 内存合并访问优化（每个线程读取连续三个坐标）
    const float x = points[idx*3];
    const float y = points[idx*3+1];
    const float z = points[idx*3+2];
    
    if (z <= min_z || z > max_z) return; // 深度过滤[1](@ref)

    // 透视投影计算（基于PCL的坐标转换模型[1](@ref)）
    const float inv_z = 1.0f / z;
    const int u = __float2int_rn(x * fx * inv_z + cx);
    const int v = __float2int_rn(y * fy * inv_z + cy);

    // 边界检查（符合Halcon坐标系规范[2](@ref)）
    if (u >=0 && u < width && v >=0 && v < height) {
        // 原子操作保留最小深度值（浮点数转整数处理）
        atomicMin(reinterpret_cast<unsigned int*>(&depth[v*width+u]), 
                __float_as_uint(z));
        //printf("%f ", z);
    }
}