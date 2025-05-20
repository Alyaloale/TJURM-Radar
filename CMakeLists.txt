cmake_minimum_required(VERSION 3.16)
project(TJURM-Radar)

# 设置编译指令
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
add_compile_options(-g)
add_compile_options(-fopenmp)

find_library(LIBUSB_LIBRARY NAMES usb-1.0)
# 设置目标兵种
add_definitions(-DTJURM_Radar)

# CUDA
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)

#TensorRT
include_directories(/home/tjurm/Downloads/TensorRT-8.4.3.1/include)

# TensorRT 库路径
link_directories(/home/tjurm/Downloads/TensorRT-8.4.3.1/targets/x86_64-linux-gnu/lib)

# OpenRM
find_package(OpenRM REQUIRED)
include_directories(${OpenRM_INCLUDE_DIRS})


# OpenCV
find_package(OpenCV 4.5.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# Serial
set(serial_DIR /usr/local/share/serial/share/serial/cmake)
link_directories(/usr/local/share/serial/lib)

# OpenMP
find_package(OpenMP REQUIRED)
include_directories(${OpenMP_REQUIRED_DIRS})


# 添加项目头文件
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/json/single_include/nlohmann)
include_directories(/usr/local/share/serial/include)

# 添加构建子目录
add_subdirectory(src/data_manager)
add_subdirectory(src/model)
add_subdirectory(src/fuse)
add_subdirectory(src/serial)
add_subdirectory(src/full_view)
# add_subdirectory(src/aruco)
# add_subdirectory(src/cudatools)

# 添加可执行文件
add_executable(TJURM-Radar src/main.cpp)

# 链接模块静态库
target_link_libraries(TJURM-Radar PRIVATE
    OpenMP::OpenMP_CXX  # 关键修复点：仅对 C++ 代码启用 OpenMP
    ${LIBUSB_LIBRARY}
    data_manager
    model
    fuse
    Serial
    full_view
    nvinfer
    nvonnxparser
    nvinfer_plugin
    ${OpenCV_LIBS}       # 确保 OpenCV 库被链接
    ${PCL_LIBRARIES}     # 确保 PCL 库被链接
    ${CERES_LIBRARIES}   # 确保 Ceres 库被链接
)
