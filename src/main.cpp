#include "data_manager/param.h"
#include "data_manager/base.h"
#include "model/init.h"
#include "fuse/fuse.h"
#include "fuse/dbscan.h"
#include "full_view/occluded_area.h"
#include "full_view/trajectory_fit.h"
#include <serial/serial.h>
#include "serial/serial_.h"
#include "aruco/aruco_locate.h"
#include <thread>


int main(int argc, char* argv[]) {
    auto param = Param::get_instance();

    // 初始化所有设备
    while(true){
        if(init_driver()) break;
    }
    //aruco_detect();
    // 初始化串口，建立接收、发送数据线程并进入
    serial_port_init();
    std::thread serial_thread_receive(serial_port_recv);
    std::thread serial_thread_send(serial_port_send);

    // 初始化模型
    auto rgb_model = RGB_MODEL::get_instance();

    // 等待外参标定结果
    while(true){
        if(extrinsic_calib()) break;
    }

    // 初始化深度图(需要外参支持), KD树
    init_depth();

    // 初始化被遮挡区域
    int camera_id = 0;
    cv::Mat image = cv::Mat(Data::camera[camera_id]->height, Data::camera[camera_id]->width, CV_8UC3, Data::camera[camera_id]->image);
    //cv::Mat undistort_image;
    //cv::undistort(image, undistort_image, Data::camera[camera_id]->intrinsic_matrix, Data::camera[camera_id]->distortion_coeffs);
    memcpy(Data::camera[camera_id]->image, image.data, Data::camera[camera_id]->height * Data::camera[camera_id]->width * 3);
    init_occlude_area();
    // 主循环
    while(true){

        //对图像去畸变(目的是为了更好的和由点云转来的深度图做融合)
        for(int i = 0; i < Data::camera.size(); i++){
            memcpy(Data::camera[i]->image, Data::camera[i]->image_buffer, Data::camera[i]->height * Data::camera[i]->width * 3);
            cv::Mat image = cv::Mat(Data::camera[i]->height, Data::camera[i]->width, CV_8UC3, Data::camera[i]->image);
            //cv::Mat undistort_image;
            //cv::undistort(image, undistort_image, Data::camera[i]->intrinsic_matrix, Data::camera[i]->distortion_coeffs);
            memcpy(Data::camera[i]->image, image.data, Data::camera[i]->height * Data::camera[i]->width * 3);
        }

        // 神经网络推理，得到装甲板的2D位置，同时注意记录相机id
        std::shared_ptr<std::vector<rm::YoloRectWithCamera>> yolo_list = std::make_shared<std::vector<rm::YoloRectWithCamera>>();
        for(int i = 0; i < Data::camera.size(); i++){
            auto yolo_list_single = rgb_model->detect_armor(i);
            yolo_list->insert(yolo_list->end(), yolo_list_single->begin(), yolo_list_single->end());
        }

        // // 激光雷达与RGB相机信息融合
        // for(int i = 0; i < Data::camera.size(); i++){
        //     Data::radar_depth[i] = PointCloud2Depth(Data::radar, Data::camera[i]);
        // }

        cv::Mat map = Data::map.clone();

        // 车辆信息清零
        for(auto &enemy : Data::enemy_info){
            enemy.is_occluded = true;
        }
        // 获取装甲板在场地坐标系下的3D坐标
        for(auto& yolo : *yolo_list){

            // FIXME: 比赛模式
            // 只处理敌方地面车辆
            if(yolo.class_id % 9 >= 6)
                continue;
            if(Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED){
                if(yolo.class_id / 9 != 0)
                    continue;
            }
            else{
                if(yolo.class_id / 9 != 1)
                    continue;
            }
            int camera_id = yolo.camera_id;
            std::vector<point> armor_3d_point;
            // 遍历当前矩形框(yolo)内所有点，计算出装甲板在场地坐标系下的坐标


            // 方法一: 单目法深度获取
            int x = yolo.box.x, y = yolo.box.y, w = yolo.box.width, h = yolo.box.height;
            //unditorm_box(x, y, w, h, Data::camera[camera_id]->intrinsic_matrix, Data::camera[camera_id]->distortion_coeffs);
            //画出矩形框
            // cv::line(image, cv::Point(x, y), cv::Point(x + w, y), cv::Scalar(0, 255, 0), 2);
            // cv::line(image, cv::Point(x, y), cv::Point(x, y + h), cv::Scalar(0, 255, 0), 2);
            // cv::line(image, cv::Point(x + w, y), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 2);
            // cv::line(image, cv::Point(x, y + h), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 2);
            if(x-1<0)x=1;
            if(y-1<0)y=1;
            for(int i = x-1; i < x + w && i < Data::camera[camera_id]->width; i++){
                for(int j = y-1; j < y + h && j < Data::camera[camera_id]->height; j++){
                    if(Data::depth[camera_id].at<double>(j, i) > 0.1){
                        double z = Data::depth_map[i][j].depth;
                        cv::Point3f armor_3d = cv::Point3f(Data::depth_map[i][j].X, Data::depth_map[i][j].Y,Data::depth_map[i][j].Z);
                        point q;
                        q.pos = armor_3d;
                        q.depth = z/1000;
                        armor_3d_point.push_back(q);
                    }
                }
            }

            // // 方法二:动态点云深度获取(适当扩大范围)
            // int x = yolo.box.x - yolo.box.width * 5, y = yolo.box.y - yolo.box.height * 5, w = yolo.box.width * 10, h = yolo.box.height * 10;
            // for(int i = x-1; i < x + w && i < Data::camera[camera_id]->width && i > 0; i++){
            //     for(int j = y-1; j < y + h && j < Data::camera[camera_id]->height && j > 0; j++){
            //         if(Data::radar_depth[camera_id].at<double>(j, i) != 0){
            //             // std::cout <<  Data::depth[camera_id].at<double>(j, i) << " ";
            //             // 1. 计算该点在相机坐标系下的坐标（利用当前像素坐标，深度信息，以及内参矩阵）
            //             double z = Data::radar_depth[camera_id].at<double>(j, i) * 1000;
            //             cv::Mat point_pixel = (cv::Mat_<double>(3, 1) << i*z, j*z, z);
            //             cv::Mat camera_cor_mat = Data::camera[camera_id]->intrinsic_matrix.inv() * point_pixel;
            //             Eigen::Vector4d camera_cor(camera_cor_mat.at<double>(0), camera_cor_mat.at<double>(1), camera_cor_mat.at<double>(2), 1);

            //             // 2. 计算该点在场地坐标系下的坐标（利用相机坐标系下的坐标，以及外参矩阵）
            //             Eigen::Vector4            searchPoint.y = world_cor(1);
            //             searchPoint.z = world_cor(2);
            //             std::vector<int> pointIdxNKNSearch(1);
            //             std::vector<float> pointNKNSquaredDistance(1);
            //             if(Data::kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
            //                 if(pointNKNSquaredDistance[0] > 20000){
            //                     cv::Point3f armor_3d = cv::Point3f(world_cor(0), world_cor(1), world_cor(2));
            //                     point p;
            //                     p.pos = armor_3d;
            //                     p.depth = z/1000;
            //                     armor_3d_point.push_back(p);
            //                 }
            //             }
            //         }
            //     }
            // }

            if(armor_3d_point.size() == 0)
                continue;

            cv::Point3f armor_3d_mean = dbscan(armor_3d_point);


            if(armor_3d_mean.z >= 1100)
                continue;

            // 车辆去重复
            if(!Data::enemy_info[yolo.class_id % 9].is_occluded)
            {
                armor_3d_mean.x = (armor_3d_mean.x + Data::enemy_info[yolo.class_id % 9].pos.x)/2;
                armor_3d_mean.y = (armor_3d_mean.y + Data::enemy_info[yolo.class_id % 9].pos.y)/2;
            }
            Data::enemy_info[yolo.class_id % 9].is_occluded= false;
            Data::enemy_info[yolo.class_id % 9].pos = armor_3d_mean;
            //std::cout<<"enemy id: " << yolo.class_id % 9 << " pos: " << Data::enemy_info[yolo.class_id % 9].pos.x << " " << Data::enemy_info[yolo.class_id % 9].pos.y << " " << Data::enemy_info[yolo.class_id % 9].pos.z << std::endl;
            Data::enemy_info[yolo.class_id % 9].last_pos = Data::enemy_info[yolo.class_id % 9].pos;
            // 记录当前时间
            Data::enemy_info[yolo.class_id % 9].last_time = std::chrono::high_resolution_clock::now();
        }

        //遮挡车辆处理

        for(auto &enemy : Data::enemy_info){

            if(enemy.is_occluded)
            {
                auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - enemy.last_time);
                cv::Point3f new_pos = enemy.last_pos;
                correct_pos(new_pos);
                enemy.pos = new_pos;
                if(duration_ms.count() > 3000)specialdetect(enemy.id, enemy);
            }
            // 将检测到的点绘制到小地图上
            int scale = 10;
            cv::Point2f armor_2d = cv::Point2f(enemy.pos.x/scale, Data::map.rows - enemy.pos.y/scale);

            // 写出类别(class_id) 注意颜色区分
            cv::Scalar color;
            if(Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED){
                color = cv::Scalar(255, 0, 0);
            }
            else{
                color = cv::Scalar(0, 0, 255);
            }
            cv::circle(map, armor_2d, 30, color, 3);
            cv::putText(map, std::to_string(enemy.id), armor_2d, cv::FONT_HERSHEY_SIMPLEX, 1, color, 5);
            if(Data::self_color == rm::ArmorColor::ARMOR_COLOR_RED){
                float temp = enemy.pos.x;
                enemy.map_pos.x = enemy.pos.y/10;
                enemy.map_pos.y = 1500 - temp/10;
            }
            else{
                float temp = enemy.pos.x;
                enemy.map_pos.x = 2800 - enemy.pos.y/10;
                enemy.map_pos.y = temp/10;
            }
        }

        // 判断是否有机会开启双倍易伤
        clock_t time;
        // 1.条件一: 不在双倍易伤状态，且当前有机会开启双倍易伤
        if(Data::radar_info.is_double_ing != 1 && Data::radar_info.is_have_chance >= 1){
            // 2.条件二: 存在正在被标记的车，且其正在掉血(不然开了双倍易伤也没用)
            for(auto& enemy : Data::enemy_info){
                if(enemy.is_debuff && enemy.is_dehealth){
                    // 如果已经触发两次了，那拜拜
                    if(Data::radar_cmd.radar_cmd == 2){
                        break;
                    }
                    // 如果已经触发一次，那看看时间有没有超过30s(因为一次双倍易伤持续时间为30s)
                    else if(Data::radar_cmd.radar_cmd == 1){
                        if(Data::game_status.game_progress == 4 && Data::game_status.stage_remain_time <= 150){
                            if((clock() - time) / CLOCKS_PER_SEC > 30){
                                std::cout << "double debuff 2!" << std::endl;
                                Data::radar_cmd.radar_cmd = 2;
                                break;
                            }
                        }
                    }
                    // 如果还没有触发过双倍易伤，则触发，同时记录当前时间
                    else{
                        if(Data::game_status.game_progress == 4 && Data::game_status.stage_remain_time <= 270){
                            std::cout << "double debuff 1!" << std::endl;
                            Data::radar_cmd.radar_cmd = 1;
                            time = clock();
                            break;
                        }
                    }
                }
            }
        }


        cv::Mat imagewithdepth = image.clone();
        // 绘制深度与RGB的叠加图
        for(int i = 0; i < Data::camera.size(); i++){
            cv::Mat depth_image = Data::depth[i];
            for(int i = 0; i < depth_image.rows; i++){
                for(int j = 0; j < depth_image.cols; j++){
                    double pixel = depth_image.at<double>(i, j);
                    //std::cout << pixel << " ";
                    // pixel, 从0到1, 红近蓝远
                    pixel /= 1000;
                    pixel = pixel < 1 ? 0 : pixel > 28.0 ? 1 : pixel/28.0;
                    if(pixel != 0){
                        imagewithdepth.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j) * 0.1 + cv::Vec3b(255*pixel, 0, 255*(1-pixel)) * 0.9;
                    }
                }
            }
            cv::resize(imagewithdepth, imagewithdepth, cv::Size(1280, 960));
            cv::imshow("image" + std::to_string(i), imagewithdepth);
        }


        // cv::resize(Data::radar_depth[0], Data::radar_depth[0], cv::Size(1280, 960));
        // cv::imshow("depth", Data::radar_depth[0]);
        cv::Mat image_show = image.clone();
        cv::resize(image_show, image_show, cv::Size(1280, 960));
        cv::imshow("image", image_show);
        // 将map缩小3倍显示
        cv::resize(map, map, cv::Size(map.cols/3, map.rows/3));
        cv::imshow("map", map);
        //q暂停
        if(cv::waitKey(1) == 'q'){
            cv::waitKey(0);
        }
        cv::waitKey(1);
    }

    return 0;
}

