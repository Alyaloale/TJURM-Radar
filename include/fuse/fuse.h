#ifndef RM2024_FUSE_FUSE_H_
#define RM2024_FUSE_FUSE_H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>
#include <algorithm>
#include <thread>


cv::Mat PointCloud2Depth(rm::Radar* radar, rm::Camera* camera);

//带颜色和深度信息点集
extern std::vector<pointwithcolor> point_set;

bool extrinsic_calib();

void init_depth();

bool cmp(std::pair<cv::Point2i,double> a, std::pair<cv::Point2i,double> b);//排序函数


bool depth_judge(std::pair<cv::Point2i,double> p,int size, double  distance);//判断深度是否在范围内


void allocate_points(int id);//多线程处理分配点集


void get_depth(std::vector<std::pair<cv::Point2i,double>> &depth_set);//多线程处理获取深度
void get_depthset(int id,Eigen::Matrix<double, 3, Eigen::Dynamic> &cloud_matrix_in_camera_3d);//多线程处理获取深度点集


void unditorm_box(int &x, int &y, int &w, int &h, const cv::Mat& K, const cv::Mat& D);//box去畸变


#endif