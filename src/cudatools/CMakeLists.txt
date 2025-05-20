# 选择是否使用静态CUDA运行时
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# 设置CUDA编译器和启用CUDA语言支持
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

# 设置 CUDA 架构
set(CMAKE_CUDA_ARCHITECTURES 7.0 8.7)

# 添加CUDA编译选项
set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-std=c++17;-g;-G;-Xcompiler;-w)


file(GLOB_RECURSE
    cuda_src
        ${CMAKE_SOURCE_DIR}/src/cudatools/*.cpp
        ${CMAKE_SOURCE_DIR}/src/cudatools/*.cu
)

# 添加头文件路径
#include_directories(${CUDA_INCLUDE_DIRS})
#include_directories(${CMAKE_SOURCE_DIR}/cuda/include)

# 编译为链接库
add_library(cudatools
    STATIC 
        ${cuda_src}
)

# 链接CUDA库
target_link_libraries(
    cudatools
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
    ${OpenRM_LIBS}
)