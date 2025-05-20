#include "aruco/aruco_locate.h"

cv::Mat getDirectionVector(const cv::Point2f &point, const cv::Mat &intrinsicMatrix, const cv::Mat &distortionCoeffs)
{
    // 计算相机坐标系下的方向向量

    // Step 1: 去畸变校正，得到归一化平面坐标
    std::vector<cv::Point2f> srcPoints = {point};
    std::vector<cv::Point2f> dstPoints;
    cv::undistortPoints(
        srcPoints, dstPoints,
        intrinsicMatrix, distortionCoeffs,
        cv::noArray()  // 不使用额外的矫正矩阵R
    );

    // Step 2: 提取去畸变后的归一化坐标
    cv::Point2f normalizedPoint = dstPoints[0];
    //std::cout<<dstPoints[0].x<<" "<<dstPoints[0].y<<std::endl;

    // Step 3: 构造相机坐标系下的方向向量 (x, y, 1)
    cv::Mat direction = (cv::Mat_<float>(3, 1)
                             << normalizedPoint.x,
                         normalizedPoint.y, 1.0f);

    // 归一化向量（若需要单位向量）
    direction /= cv::norm(direction);

    return direction;
}

cv::Point3f direction2place(const cv::Mat &direction, cv::Point3f &origin, float targetY) {
    // 提取方向向量的分量
    float dx = direction.at<float>(0);
    float dy = direction.at<float>(1);
    float dz = direction.at<float>(2);

    // 处理方向向量x分量为零的情况
    const float epsilon = 1e-6f;
    if (std::abs(dy) < epsilon) {
        if (std::abs(origin.x - targetY) < epsilon) {
            // 直线位于x平面内，返回原点（此时直线与平面有无数交点）
            return origin;
        } else {
            // 无交点，返回(0,0,0)表示无效
            return cv::Point3f(0, 0, 0);
        }
    }

    // 计算参数t
    float t = (targetY - origin.y) / dy;

    // 计算交点的y和z坐标
    float x = origin.x + dx * t;
    float z = origin.z + dz * t;

    // 返回交点，x坐标固定为targetX
    return cv::Point3f(x, targetY, z);
}


// 主判断函数
bool isPointInPolygon(const std::vector<cv::Point2f>& polygon, const cv::Point2f& P) {
    if (polygon.size() < 3) return false;

    // 阶段1：边界检查
    for (size_t i=0; i<polygon.size(); ++i) {
        const auto& A = polygon[i];
        const auto& B = polygon[(i+1)%polygon.size()];
        if (isPointOnSegment(P, A, B)) return true;
    }

    // 阶段2：射线法核心逻辑
    int intersectionCount = 0;
    const double py = P.y;
    const double px = P.x;
    
    for (size_t i=0; i<polygon.size(); ++i) {
        const auto& A = polygon[i];
        const auto& B = polygon[(i+1)%polygon.size()];
        
        // 提升到双精度计算
        const double ay = A.y, by = B.y;
        const double ax = A.x, bx = B.x;

        // 跳过水平边
        if (std::abs(by - ay) < EPSILON) continue;

        // 计算交点参数
        const double t = (py - ay) / (by - ay);
        if (t < 0.0 || t > 1.0) continue;

        // 计算交点x坐标
        const double x_intersect = ax + t * (bx - ax);

        // 统计右侧交点（包含误差补偿）
        if (x_intersect > px - EPSILON) {
            ++intersectionCount;
        }
    }

    return (intersectionCount % 2) == 1;
}