#include "data_manager/param.h"
#include "data_manager/base.h"
#include "full_view/occluded_area.h"
#include <openrm/cudatools.h>
#include <unistd.h>
#include <algorithm>
void get_occlude_area()
{
    auto param = Param::get_instance();
    //图片大小控制比率
    int kx = (*param)["OccludedArea"]["KernelSize"];
    //场地和图片坐标系的比例
    int ky = 28000/(28*kx);
    Data::occluded_area = cv::Mat::zeros(15*kx, 28*kx, CV_8UC3);
    
    memcpy(Data::camera[0]->image, Data::camera[0]->image_buffer, Data::camera[0]->height * Data::camera[0]->width * 3);
    cv::Mat image = cv::Mat(Data::camera[0]->height, Data::camera[0]->width, CV_8UC3, Data::camera[0]->image);
    //cv::imshow("image", image);

    //将场地坐标系下的点集投影到俯视图
    #pragma omp parallel for num_threads(Data::thread_num)
    for(int i = 0; i < Data::point_set.size(); i++){
        int x = (Data::point_set[i].X/ky-int(Data::point_set[i].X/ky))>=0.5?Data::point_set[i].X/ky+1:Data::point_set[i].X/ky;
        int y = (Data::point_set[i].Y/ky-int(Data::point_set[i].Y/ky))>=0.5?Data::point_set[i].Y/ky+1:Data::point_set[i].Y/ky;
        if(x>=0&&y>=0&&x<15*kx&&y<28*kx){
            Data::point_set[i].color=image.at<cv::Vec3b>(Data::point_set[i].y,Data::point_set[i].x);
            Data::occluded_area.at<cv::Vec3b>(x, y) = Data::point_set[i].color;
        }
    }
    cv::Mat occlude_clone = Data::occluded_area.clone();
    cv::resize(occlude_clone,occlude_clone,cv::Size(1280,960));
    //显示图像
    cv::imshow("occluded_area", occlude_clone);
}

//遮挡区域获取
void repair_occlude_area()
{

    cv::Mat area;
    cv::cvtColor(Data::occluded_area, area, cv::COLOR_BGR2GRAY);
    //反二值化
    cv::Mat binary;
    cv::threshold(area, binary, 0, 255, cv::THRESH_BINARY_INV);
    //cv::imshow("binary", binary);
    cv::Mat binary1 = binary.clone();
    //插值修复
    int size=3;
    for(int i = 0; i < Data::occluded_area.cols; i++){
        for(int j = 0; j < Data::occluded_area.rows; j++){
            if(binary.at<uchar>(j, i) == 255){
                int x1 = i-size<0?0:i-size;
                int x2 = i+size>=Data::occluded_area.cols?Data::occluded_area.cols-1:i+size;
                int y1 = j-size<0?0:j-size;
                int y2 = j+size>=Data::occluded_area.rows?Data::occluded_area.rows-1:j+size;
                int cnt=0;
                for(int x = x1; x <= x2; x++){
                    for(int y = y1; y <= y2; y++){
                        if(binary.at<uchar>(y, x) == 255)cnt++;
                    }
                }
                double rate = double(cnt)/((x2-x1+1)*(y2-y1+1));
                if(rate<0.95)binary1.at<uchar>(j, i) = 0;
                else binary1.at<uchar>(j, i) = 255;
            }
        }
    }
    //cv::imshow("binary1", binary1);
    bfs_getarea(binary1);
}
void bfs_getarea(cv::Mat &binary)
{
    //初始化数据
    Data::area_point.resize(binary.cols);
    Data::is_beoccluded.resize(binary.cols);
    for(int i = 0; i < binary.cols; i++){
        Data::area_point[i].resize(binary.rows);
        Data::is_beoccluded[i].resize(binary.rows);
    }

    //bfs搜索连通区域
    int dx[4] = {0, 0, 1, -1};
    int dy[4] = {1, -1, 0, 0};
    int area_num = 0;
    std::queue<cv::Point2i> q;
    for(int i = 0; i < binary.cols; i++){
        for(int j = 0; j < binary.rows; j++){
            if(binary.at<uchar>(j, i) == 255 && !Data::is_beoccluded[i][j]){
                area_num++;
                q.push(cv::Point2i(i, j));
                area ar;
                Data::area_point[i][j] = area_num;
                Data::is_beoccluded[i][j] = true;
                while(!q.empty()){
                    cv::Point2i p = q.front();
                    q.pop();
                    ar.point.push_back(p);
                    for(int k = 0; k < 4; k++){
                        int x = p.x + dx[k];
                        int y = p.y + dy[k];
                        if(x >= 0 && x < binary.cols && y >= 0 && y < binary.rows && binary.at<uchar>(y, x) == 255 && Data::is_beoccluded[x][y] == false){
                            q.push(cv::Point2i(x, y));
                            Data::area_point[x][y] = area_num;
                            Data::is_beoccluded[x][y] = true;
                        }
                    }
                }
                if(ar.point.size() > 400)Data::areas.push_back(ar);
                else{
                    for(int k = 0; k < ar.point.size(); k++){
                        binary.at<uchar>(ar.point[k].y, ar.point[k].x) = 0;
                        Data::is_beoccluded[ar.point[k].x][ar.point[k].y] = false;
                        Data::area_point[ar.point[k].x][ar.point[k].y] = 0;
                    }
                }
            }
        }
    }
    //把每个区域以不同颜色显示
    // cv::Mat show = cv::Mat::zeros(binary.rows, binary.cols, CV_8UC3);
    // for(int i = 0 ;i < Data::areas.size(); i++){
    //     for(int j = 0; j < Data::areas[i].point.size(); j++){
    //         //不同颜色红绿蓝黄轮流显示
    //         int color = i % 4;
    //         switch(color){
    //             case 0:
    //                 show.at<cv::Vec3b>(Data::areas[i].point[j].y, Data::areas[i].point[j].x) = cv::Vec3b(255, 0, 0);
    //                 break;
    //             case 1:
    //                 show.at<cv::Vec3b>(Data::areas[i].point[j].y, Data::areas[i].point[j].x) = cv::Vec3b(0, 255, 0);
    //                 break;
    //             case 2:
    //                 show.at<cv::Vec3b>(Data::areas[i].point[j].y, Data::areas[i].point[j].x) = cv::Vec3b(0, 0, 255);
    //                 break;
    //             case 3:
    //                 show.at<cv::Vec3b>(Data::areas[i].point[j].y, Data::areas[i].point[j].x) = cv::Vec3b(255, 255, 0);
    //                 break;
    //         }
    //     }
    // }
    //cv::imshow("show", show);

}
void init_occlude_area()
{
    get_occlude_area();
    repair_occlude_area();
}


void correct_pos(cv::Point3f& new_pos)
{
    int px=(new_pos.x/10-int(new_pos.x/10))>=0.5?new_pos.x/10+1:new_pos.x/10;
    int py=(new_pos.y/10-int(new_pos.y/10))>=0.5?new_pos.y/10+1:new_pos.y/10;
    std::vector<cv::Point2d>may_points;
    for(int i = -400; i <= 400; i++){
        for(int j = -400; j <= 400; j++){
            int x = int(px + i + 0.5);
            int y = int(py + j + 0.5);
            if(x>=0&&y>=0&&x<1500&&y<2800&&Data::is_beoccluded[x][y]){
                may_points.push_back(cv::Point2d(x, y));
            }
        }
    }
    cv::Point2f best_point;
    for(int i = 0; i < may_points.size(); i++){
        best_point.x += may_points[i].x;
        best_point.y += may_points[i].y;
    }
    best_point.x /= may_points.size();
    best_point.y /= may_points.size();

    //修正new_pos
    new_pos.x = best_point.x*10;
    new_pos.y = best_point.y*10;
}
void specialdetect(int id,Car &car)
{
    if(car.pos.y > 24350 && car.pos.x < 5450)
    {
        car.pos.x = 1180.18;
        car.pos.y = 25349;
    }
    else if(car.pos.y < 24993 &&car.pos.y > 22519 && car.pos.x > 9516)
    {
        car.pos.x = 11743.18;
        car.pos.y = 23706;
    }
    else if(car.pos.y < 22519&& car.pos.y >17868 )
    {
        if(car.pos.x < 7517)
        {
            if(id==2)
            {
                if(car.pos.x < 5700)
                {
                    if(car.pos.x > 5110)
                    {
                        car.pos.x = 5110;
                        car.pos.y = 22075;
                    }
                }
                else
                {
                    car.pos.x = 4320;
                    car.pos.y = 18328;
                }
            }
            else
            {
                car.pos.x = 4320;
                car.pos.y = 18328;
            }
        }
        else
        {
            if(id==2)
            {
                if(car.pos.x < 9153)
                {
                    car.pos.x = 8630;
                    car.pos.y = 18721.28;
                }
                else
                {
                    car.pos.x = 12274.02;
                    car.pos.y = 16704.4;
                }
            }
            else
            {
                car.pos.x = 11257.02;
                car.pos.y = 17344.4;
            }
        }
    }
    else if(car.pos.y > 14018 && car.pos.y < 17868 && car.pos.x > 2548 && car.pos.x < 12487)
    {
        car.pos.x = 7634;
        car.pos.y = 15681.67;
    }
    else if(car.pos.y > 9107 && car.pos.y < 14018)
    {
        if(car.pos.x < 4218)
        {
            car.pos.x = 1913;
            car.pos.y = 12027;
        }
    }
}