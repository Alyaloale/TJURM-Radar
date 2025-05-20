#include "cudatools/cudatool.h"



void depth_to_pointcloud(std::vector<float>* &points, cv::Mat& depth,
    float fx, float fy, float cx, float cy, float scale) {
    int width = depth.cols;
    int height = depth.rows;

    // 分配 GPU 内存
    float* d_depth;
    float* d_points;
    cudaMalloc((void**)&d_depth, width * height * sizeof(float));
    cudaMalloc((void**)&d_points, width * height * 3 * sizeof(float));

    // 将深度图数据复制到 GPU
    cudaMemcpy(d_depth, depth.ptr<float>(), width * height * sizeof(float), cudaMemcpyHostToDevice);

    // 定义 CUDA 网格和块大小
    dim3 blockSize(16, 16); // 每个块有 16x16 个线程
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);

    // 调用 CUDA 核函数
    depth_to_pointcloud_kernel<<<gridSize, blockSize>>>(d_points, d_depth, width, height, fx, fy, cx, cy, scale);

    // 将结果从 GPU 复制回 CPU
    points->resize(width * height * 3);
    cudaMemcpy(points->data(), d_points, width * height * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    // 释放 GPU 内存
    cudaFree(d_depth);
    cudaFree(d_points);
}


void point_transform(std::vector<float>* &points,
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& T) {
    size_t num_points = points->size() / 3;

    // 分配 GPU 内存
    float* d_points;
    float* d_R;
    float* d_T;
    cudaMalloc((void**)&d_points, points->size() * sizeof(float));
    cudaMalloc((void**)&d_R, 9 * sizeof(float)); // 3x3 旋转矩阵
    cudaMalloc((void**)&d_T, 3 * sizeof(float)); // 3x1 平移向量

    // 将数据复制到 GPU
    cudaMemcpy(d_points, points->data(), points->size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_R, R.data(), 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_T, T.data(), 3 * sizeof(float), cudaMemcpyHostToDevice);

    // 定义 CUDA 网格和块大小
    int blockSize = 256; // 每个块有 256 个线程
    int gridSize = (num_points + blockSize - 1) / blockSize;

    // 调用 CUDA 核函数
    point_transform_kernel<<<gridSize, blockSize>>>(d_points, num_points, d_R, d_T);

    // 将结果从 GPU 复制回 CPU
    cudaMemcpy(points->data(), d_points, points->size() * sizeof(float), cudaMemcpyDeviceToHost);

    // 释放 GPU 内存
    cudaFree(d_points);
    cudaFree(d_R);
    cudaFree(d_T);
}

// 点云转深度图
void project_pointcloud_to_depth(
    std::vector<float>* &points, 
    std::vector<float>* &depth,
    float fx, float fy, float cx, float cy,
    int width, int height)
{
    const size_t num_points = points->size() / 3;
    depth->resize(width * height);

    // GPU内存管理（PCL兼容的内存布局[1](@ref)）
    float *d_points = nullptr, *d_depth = nullptr;
    cudaMalloc(&d_points, points->size() * sizeof(float));
    cudaMalloc(&d_depth, width*height*sizeof(float));

    // 初始化深度图为FLT_MAX（Halcon无效值规范[2](@ref)）
    cudaMemset(d_depth, 0xFF, width*height*sizeof(float));
    
    // 数据传输（异步优化）
    cudaMemcpy(d_points, points->data(), 
              points->size()*sizeof(float),
              cudaMemcpyHostToDevice);

    // 内核启动配置（基于NVIDIA最佳实践）
    const int blockSize = 256;  // 根据GPU架构调整
    const int gridSize = (num_points + blockSize - 1) / blockSize;
    
    // 执行投影计算（包含深度范围过滤[1](@ref)）
    project_pointcloud_to_depth_kernel<<<gridSize, blockSize>>>(
        d_points, num_points, d_depth, 
        width, height, fx, fy, cx, cy,
        100.0f, 10000.0f); // 默认深度范围0.1-10米
    
    // 数据回传
    cudaMemcpy(depth->data(), d_depth, width*height*sizeof(float),
              cudaMemcpyDeviceToHost);

    // 后处理：替换无效值为0（Halcon规范[2](@ref)）
    for (auto& val : *depth) {
        if (*(uint32_t*)&val == 0xFFFFFFFF) val = 0.0f;
    }

    // 资源释放
    cudaFree(d_points);
    cudaFree(d_depth);
}