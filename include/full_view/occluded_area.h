#ifndef RM2024_FULL_VIEW_OCCLUDED_AREA_H_
#define RM2024_FULL_VIEW_OCCLUDED_AREA_H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>



void get_occlude_area();//获取遮挡区域

void init_occlude_area();//初始化遮挡区域

void repair_occlude_area();//修复遮挡区域

cv::Mat deleteMinWhiteArea(cv::Mat src,int min_area);//删除最小白色区域
void bfs_getarea(cv::Mat &binary);

void correct_pos(cv::Point3f& new_pos);//根据被遮挡区域修正卡尔曼

void specialdetect(int id,Car &pos);//特殊检测


#endif