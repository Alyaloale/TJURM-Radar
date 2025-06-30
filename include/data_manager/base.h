#ifndef RM2024_DATA_MANAGER_BASE_H_
#define RM2024_DATA_MANAGER_BASE_H_

#include <opencv2/opencv.hpp>
#include <openrm.h>
#include <Eigen/Dense>
#include <cstdint>
#include <serial/serial.h>
#include "serial/serial_.h"
// pcl库
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <omp.h>
#include <opencv2/aruco.hpp>



// 存储地敌方车辆信息 0哨兵 1-5同车辆贴纸数字
struct Car {
    int id;
    cv::Point3f last_pos;
    cv::Point3f map_pos;
    cv::Point3f pos;// 场地坐标系下的坐标
    bool is_debuff;     // 是否正在debuff状态
    bool is_dehealth;   // 是否正在掉血
    bool is_occluded;   // 是否被遮挡
    bool is_live;       // 是否存活
        //std::vector<cv::Point3f>possible_location;//可能的位置
};

struct pointwithcolor{
    double depth;//深度信息
    double X,Y,Z;//场地坐标系下的坐标
    cv::Vec3b color;//颜色信息
    int x,y;//图像坐标系下的坐标
};

struct area{
    std::vector<cv::Point2i> point;//遮挡区域点集
};


namespace Data {
    // time
    extern game_status_t game_status;

    // 颜色
    extern rm::ArmorColor self_color;
    extern rm::ArmorColor enemy_color;

    // 敌方车信息
    extern std::vector<Car> enemy_info;

    // 雷达标记进度数据
    extern radar_mark_data_t radar_mark_data;
    extern radar_info_t radar_info;
    extern robot_interaction_data_t robot_interaction_data;
    extern game_robot_HP_t game_robot_HP;

    // 雷达标记数据
    extern map_robot_data_t map_robot_data;
    extern radar_cmd_t radar_cmd;

    // 相机的参数记录
    extern std::vector<rm::Camera*> camera;
    extern rm::Radar* radar;

    // 点云转换得到的深度图(便于信息融合)
    extern std::vector<cv::Mat> radar_depth;
    extern std::vector<cv::Mat> depth;
    extern std::vector<pointwithcolor> point_set;

    // 外参标定结果(雷达站到场地坐标系的，以及每个相机到场地坐标系的)
    extern rm::RadarData* extrinsic;
    extern Eigen::Matrix<double, 4, 4> radar2place;
    extern std::vector<Eigen::Matrix<double, 4, 4>> camera2place;

    // 敌方6辆车的位置
    extern std::vector<cv::Point3f> enemy_pos;

    // 串口
    extern serial::Serial ser;

    // 小地图map
    extern cv::Mat map;

    // KD树+八叉树
    extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    extern pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // extern pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

    // 带场地坐标的场地图
    extern std::pair<cv::Mat, cv::Mat> image_with_coordinate;


    extern cv::Mat occluded_area;//含被遮挡区域场地俯视图
    extern cv::Mat occluded_area_depth;//未经过深度处理的遮挡区域深度图
    extern std::vector<area>areas;//遮挡区域
    extern std::vector<std::vector<int> >area_point;//点对应的遮挡区域
    extern std::vector<std::vector<bool> >is_beoccluded;//点是否被遮挡

    extern std::vector<std::vector<pointwithcolor>> depth_map;//深度点集
    extern int thread_num;

    extern std::vector<int> markerIds;//标记点id
    extern std::vector<std::vector<cv::Point2f>> markerCorners;//标记点角点
    extern std::vector<std::vector<cv::Point2f>> rejectedCandidates;//被拒绝的候选点
    extern cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName;//字典名称
}

bool init_driver();

#endif