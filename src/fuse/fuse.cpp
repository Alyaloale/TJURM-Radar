#include "data_manager/param.h"
#include "data_manager/base.h"
#include "fuse/fuse.h"
#include <openrm/cudatools.h>
#include <unistd.h>
#include <mutex>
//camera[0] HiK
int camera_id = 0;
//图片
cv::Mat image;
//深度点集
std::vector<std::vector<std::pair<cv::Point2i,double>> > depth_sets(Data::thread_num);
std::vector<std::pair<cv::Point2i,double>> depth_set;
//锁
std::mutex mtx_depth_sets, mtx_depmap, mtx_depset;


cv::Mat PointCloud2Depth(rm::Radar* radar, rm::Camera* camera){
    cv::Mat close_image = cv::Mat(camera->height, camera->width, CV_8UC3, camera->image);
    cv::Mat radar_image = cv::Mat(4, radar->num_point, CV_32FC1, radar->point_cloud_buffer);
    // FIXME: 注意这里拿到的点云单位是m

    // 1. 为了方便，先将点云数据转换Matrix格式
    Eigen::Matrix<double, 4, Eigen::Dynamic> radar_matrix = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, radar->num_point);
    for(int i = 0; i < radar->num_point; i++){
        radar_matrix(0, i) = radar_image.at<float>(0, i);
        radar_matrix(1, i) = radar_image.at<float>(1, i);
        radar_matrix(2, i) = radar_image.at<float>(2, i);
        radar_matrix(3, i) = 1;
    }

    // 2. 将点云经过camera的联合标定参数转换到camera坐标系下
    Eigen::Matrix<double, 4, Eigen::Dynamic> radar_matrix_in_close = camera->Trans_pnp2head * radar_matrix;

    // 3. 将点云投影到camera的图像坐标系下
    Eigen::Matrix<double, 3, 3> intrinsic_matrix;
    rm::tf_Mat3d(camera->intrinsic_matrix, intrinsic_matrix);
    Eigen::Matrix<double, 3, Eigen::Dynamic> radar_matrix_in_close_3d = intrinsic_matrix * radar_matrix_in_close.topRows(3);

    // 4. 将点云投影到图像坐标系下，得到深度图
    cv::Mat depth_image = cv::Mat::zeros(camera->height, camera->width, CV_64FC1);
    for(int i = 0; i < radar->num_point; i++){
        int x = radar_matrix_in_close_3d(0, i) / radar_matrix_in_close_3d(2, i);
        int y = radar_matrix_in_close_3d(1, i) / radar_matrix_in_close_3d(2, i);
        if(x >= 0 && x < camera->width && y >= 0 && y < camera->height){
            double depth = radar_matrix_in_close_3d(2, i);
            depth_image.at<double>(y, x) = depth;
        }
    }
    return depth_image;
}


bool extrinsic_calib(){
    auto param = Param::get_instance();
    
    // 等待人工选点结果
    if(!Data::extrinsic->is_valid){
        return false;
    }
    Data::radar2place = Eigen::Matrix<double, 4, 4>::Identity();


    // 对每个相机进行外参标定，得到外参标定结果
    for(int i = 0; i < Data::camera.size(); i++){
        // 1. 获取世界坐标（从配置文件里读取）
        std::vector<cv::Point3d> zed_cali(4);
        std::string key = "PlacePoint" + std::to_string(i);
        std::cout << "PlaceName: " << (*param)["PlaceName"+ std::to_string(i)] << std::endl;
        zed_cali[0].x = (*param)[key]["1x"];
        zed_cali[0].y = (*param)[key]["1y"];
        zed_cali[0].z = (*param)[key]["1z"];
        zed_cali[1].x = (*param)[key]["2x"];
        zed_cali[1].y = (*param)[key]["2y"];
        zed_cali[1].z = (*param)[key]["2z"];
        zed_cali[2].x = (*param)[key]["3x"];
        zed_cali[2].y = (*param)[key]["3y"];
        zed_cali[2].z = (*param)[key]["3z"];
        zed_cali[3].x = (*param)[key]["4x"];
        zed_cali[3].y = (*param)[key]["4y"];
        zed_cali[3].z = (*param)[key]["4z"];

        // 2. 获取相机坐标
        std::vector<cv::Point2d> radar_cali(4);
        for(int j = 0; j < 4; j++){
            radar_cali[j].x = Data::extrinsic->image_zed_calib[j].x * Data::camera[i]->width;
            radar_cali[j].y = Data::extrinsic->image_zed_calib[j].y * Data::camera[i]->height;
        }
        // std::cout<<"radar_cali: " << radar_cali[0].x << " " << radar_cali[0].y << std::endl;
        // std::cout<<"zed_cali: " << zed_cali[0].x << " " << zed_cali[0].y << std::endl;
        // std::cout<<"radar_cali: " << radar_cali[1].x << " " << radar_cali[1].y << std::endl;
        // std::cout<<"zed_cali: " << zed_cali[1].x << " " << zed_cali[1].y << std::endl;
        // std::cout<<"radar_cali: " << radar_cali[2].x << " " << radar_cali[2].y << std::endl;
        // std::cout<<"zed_cali: " << zed_cali[2].x << " " << zed_cali[2].y << std::endl;
        // std::cout<<"radar_cali: " << radar_cali[3].x << " " << radar_cali[3].y << std::endl;
        // std::cout<<"zed_cali: " << zed_cali[3].x << " " << zed_cali[3].y << std::endl;
        //设定估计内参
        // cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        // 3250, 0, 2048,
        // 0, 3250, 1230,
        // 0, 0, 1);

        // 3. 使用solvepnp求解相机与场地坐标系的外参
        cv::Mat rvec, tvec;
        //cv::solvePnP(zed_cali, radar_cali, camera_matrix, Data::camera[i]->distortion_coeffs, rvec, tvec, 0, cv::SOLVEPNP_EPNP);
        cv::solvePnP(zed_cali, radar_cali, Data::camera[i]->intrinsic_matrix, Data::camera[i]->distortion_coeffs, rvec, tvec, 0, cv::SOLVEPNP_EPNP);

        // 4. 将rvec和tvec转换为4x4的矩阵place2camera
        Eigen::Matrix<double, 4, 4> place2camera = Eigen::Matrix<double, 4, 4>::Identity();
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix<double, 3, 3> rotate;
        rm::tf_Mat3d(rmat, rotate);
        Eigen::Matrix<double, 4, 1> pose;
        rm::tf_Vec4d(tvec, pose);
        rm::tf_rt2trans(pose, rotate, place2camera);
        Data::camera2place.push_back(place2camera.inverse());

        // // 5. 得到radar2place
        Data::radar2place = place2camera.inverse() * Data::camera[0]->Trans_pnp2head;
    }


    // 输出tvec 
    std::cout << "x: " << Data::camera2place[0](0, 3) << " y: " << Data::camera2place[0](1, 3) << " z: " << Data::camera2place[0](2, 3) << std::endl;
    //std::cout<<Data::camera2place[0];
    
    return true;
}


void init_depth(){
    //初始化数据
    Data::depth_map.resize(Data::camera[camera_id]->width);
    for(int i = 0; i < Data::camera[camera_id]->width; i++){
        Data::depth_map[i].resize(Data::camera[camera_id]->height);
    }


    // 读取pcl文件，拿到PointCloud格式的点云数据
    auto param = Param::get_instance();
    //记录此时时间

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Data::cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd_path = (*param)["PointCloud"]["Dir"];
    pcl::io::loadPCDFile(pcd_path, *Data::cloud);
    //FIXME: 注意这里拿到的点云单位是mm


    
    Eigen::setNbThreads(Data::thread_num);
    Eigen::initParallel();
    //1.将点云数据转换为Matrix格式
    Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_matrix = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, Data::cloud->width);
    #pragma omp parallel for num_threads(Data::thread_num)
    for(int i = 0; i < Data::cloud->width; i++){
        cloud_matrix(0, i) = Data::cloud->points[i].x;
        cloud_matrix(1, i) = Data::cloud->points[i].y;
        cloud_matrix(2, i) = Data::cloud->points[i].z;
        cloud_matrix(3, i) = 1;
    }


    //2.将点云投影到雷达站的图像坐标系下，根据各相机的外参，将点云转换到相机坐标系下
    Eigen::Matrix<double, 4,Eigen::Dynamic> cloud_matrix_in_camera;
    cloud_matrix_in_camera = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, Data::cloud->width);
    #pragma omp parallel for num_threads(Data::thread_num)
    for(int i = 0; i < Data::cloud->width; i++){
        cloud_matrix_in_camera.col(i) = Data::camera2place[camera_id].inverse() * cloud_matrix.col(i);
    }
    //Eigen::Matrix<double, 4, Eigen::Dynamic> cloud_matrix_in_camera = Data::camera2place[camera_id].inverse() * cloud_matrix;
    Eigen::Matrix<double, 3, 3> intrinsic_matrix;
    rm::tf_Mat3d(Data::camera[camera_id]->intrinsic_matrix, intrinsic_matrix);
    //Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_matrix_in_camera_3d = intrinsic_matrix * cloud_matrix_in_camera.topRows(3);
    Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_matrix_in_camera_3d;
    cloud_matrix_in_camera_3d = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, Data::cloud->width);
    #pragma omp parallel for num_threads(Data::thread_num)
    for(int i=0; i < Data::cloud->width; i++)
    {
        cloud_matrix_in_camera_3d.col(i) = intrinsic_matrix * cloud_matrix_in_camera.topRows(3).col(i);
    }


    //多线程处理
    std::thread threads[Data::thread_num];

    //3.将点云投影到图像坐标系下，并筛选得到深度图
    Data::occluded_area_depth = cv::Mat::zeros(Data::camera[camera_id]->height, Data::camera[camera_id]->width, CV_64FC1);
    for(int i = 0;i <Data::thread_num;i++){
        threads[i] = std::thread(get_depthset,i,std::ref(cloud_matrix_in_camera_3d));
    }
    for(int i = 0;i < Data::thread_num;i++){
        threads[i].join();
    }


    //4.深度筛选

    for(int i = 0;i <Data::thread_num;i++){
        threads[i] = std::thread(allocate_points,i);
    }
    for(int i = 0;i < Data::thread_num;i++){
        threads[i].join();
    }
    for(int i = 0;i < Data::thread_num;i++){

        threads[i] = std::thread(get_depth,std::ref(depth_sets[i]));
    }
    for(int i = 0;i < Data::thread_num;i++){
        threads[i].join();
    }
    Data::depth.push_back(Data::occluded_area_depth);

}
bool cmp(std::pair<cv::Point2i,double> a,std::pair<cv::Point2i,double> b){
    return a.second < b.second;
}

//判断深度是否在范围内
bool depth_judge(std::pair<cv::Point2i,double> p,int size, double  distance)
{
    int x=p.first.x;
    int y=p.first.y;
    int count = 0;
    double avreage_depth = 0, sum = 0;
    int x1=(x-size)>=0?(x-size):0;
    int x2=x+size<Data::camera[0]->width?x+size:Data::camera[0]->width;
    int y1=(y-size)>=0?(y-size):0;
    int y2=y+size<Data::camera[0]->height?y+size:Data::camera[0]->height;
    for(int i = x1; i < x2; i++){
        for(int j = y1; j < y2; j++){
            if(Data::occluded_area_depth.at<double>(j, i) != 0){
                sum += Data::occluded_area_depth.at<double>(j, i);
                count++;
            }
        }
    }
    avreage_depth = sum / count;
    if(count == 0||p.second > avreage_depth - distance &&p.second < avreage_depth + distance){
        return true;
    }
    return false;
}

//多线程处理分配点集
void allocate_points(int id)
{

    for(int i = id * depth_set.size() / Data::thread_num; i < (id + 1) * depth_set.size() / Data::thread_num; i++){
            int x = depth_set[i].first.x;
            int y = depth_set[i].first.y;
            int kx = x/(Data::camera[camera_id]->width/4);
            int ky = y/(Data::camera[camera_id]->height/4);
            int k = kx*4+ky;
            mtx_depth_sets.lock();
            depth_sets[k].push_back(std::make_pair(cv::Point2i(x,y),depth_set[i].second));
            mtx_depth_sets.unlock();
        }
}


//多线程处理获取深度
void get_depth(std::vector<std::pair<cv::Point2i,double>> &depth_thread)
{
    std::sort(depth_thread.begin(), depth_thread.end(),cmp);
    for(int j = 0; j < depth_thread.size(); j++){

        int x = depth_thread[j].first.x;
        int y = depth_thread[j].first.y;
        double z = depth_thread[j].second;
        if(Data::occluded_area_depth.at<double>(y, x) !=0||!depth_judge(depth_thread[j], 3 ,150))continue;

        //计算该点在相机坐标系下的坐标（利用当前像素坐标，深度信息，以及内参矩阵）
        cv::Mat point_pixel = (cv::Mat_<double>(3, 1) << x*z, y*z, z);
        cv::Mat camera_cor_mat = Data::camera[camera_id]->intrinsic_matrix.inv() * point_pixel;
        Eigen::Vector4d camera_cor(camera_cor_mat.at<double>(0), camera_cor_mat.at<double>(1), camera_cor_mat.at<double>(2), 1);

        //计算该点在场地坐标系下的坐标（利用相机坐标系下的坐标，以及外参矩阵）
        Eigen::Vector4d world_cor = Data::camera2place[0] * camera_cor;
        mtx_depmap.lock();
        pointwithcolor p;
        p.depth = z;
        p.X = world_cor(0);
        p.Y = world_cor(1);
        p.Z = world_cor(2);
        p.x = x;
        p.y = y;
        Data::occluded_area_depth.at<double>(y, x) = z;
        Data::point_set.push_back(p);
        Data::depth_map[x][y]=p;
        mtx_depmap.unlock();
    }
}
void get_depthset(int id,Eigen::Matrix<double, 3, Eigen::Dynamic> &cloud_matrix_in_camera_3d)
{
    for(int j = id * Data::cloud->width / Data::thread_num; j < (id + 1) * Data::cloud->width / Data::thread_num; j++){
        int x = cloud_matrix_in_camera_3d(0, j) / cloud_matrix_in_camera_3d(2, j);
        int y = cloud_matrix_in_camera_3d(1, j) / cloud_matrix_in_camera_3d(2, j);
        double depth = cloud_matrix_in_camera_3d(2, j);
        if(x >= 0 && x < Data::camera[camera_id]->width && y >= 0 && y < Data::camera[camera_id]->height)
        {
            if(depth < 0)continue;
            mtx_depset.lock();
            depth_set.push_back(std::make_pair(cv::Point2i(x,y),depth));
            mtx_depset.unlock();
        }
    }
}


void unditorm_box(int &x, int &y, int &w, int &h, const cv::Mat& K, const cv::Mat& D)
{
        // 提取四个角点
    std::vector<cv::Point2f> originalCorners = {
        cv::Point2f(x, y),             // 左上
        cv::Point2f(x + w, y),         // 右上
        cv::Point2f(x, y + h),         // 左下
        cv::Point2f(x + w, y + h)      // 右下
    };

    // 去畸变校正
    std::vector<cv::Point2f> correctedCorners;
    cv::undistortPoints(originalCorners, correctedCorners, K, D, cv::noArray(), K);

    // 计算校正后的包围框
    float minX = correctedCorners[0].x, maxX = correctedCorners[0].x;
    float minY = correctedCorners[0].y, maxY = correctedCorners[0].y;
    for (const auto& p : correctedCorners) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
    }

    // 转换为整数并确保不越界
    int imgWidth = Data::camera[0]->width; // 替换为实际图像宽度
    int imgHeight = Data::camera[0]->height; // 替换为实际图像高度
    int newX = cv::saturate_cast<int>(std::round(minX));
    int newY = cv::saturate_cast<int>(std::round(minY));
    int newW = cv::saturate_cast<int>(std::round(maxX - minX));
    int newH = cv::saturate_cast<int>(std::round(maxY - minY));

    // 边界约束
    newX = std::max(0, std::min(newX, imgWidth - 1));
    newY = std::max(0, std::min(newY, imgHeight - 1));
    newW = std::max(0, std::min(newW, imgWidth - newX));
    newH = std::max(0, std::min(newH, imgHeight - newY));

    // 更新包围框
    x = newX;
    y = newY;
    w = newW;
    h = newH;
}