#include "full_view/trajectory_fit.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <functional>
using namespace std;
using namespace Eigen;

Polynomial poly;//多项式函数
// 一元多项式回归函数
void polynomialRegression(vector<cv::Point3f> &location, int degree) {
    int n = location.size();
    MatrixXd X(n, degree + 1);
    VectorXd Y(n);

    // 构造矩阵
    for (int i = 0; i < n; i++) {
        Y(i) = location[i].x;
        for (int j = 0; j <= degree; j++) {
            X(i, j) = pow(location[i].y, j);
        }
    }

    // 最小二乘法求解
    VectorXd coefficients = X.colPivHouseholderQr().solve(Y);

    // 将结果转换为std::vector
    vector<double> result(coefficients.data(), coefficients.data() + coefficients.size());
    poly.update(result);
}


// 定义数值积分函数（辛普森法）
double simpsonIntegration(const std::function<double(double)>& func, double a, double b, int n) {
    if (n % 2 != 0) n++; // 辛普森法要求分割数为偶数
    double h = (b - a) / n;
    double s = func(a) + func(b);

    for (int i = 1; i < n; i++) {
        double x = a + h * i;
        if (i % 2 == 0) {
            s += 2 * func(x);
        } else {
            s += 4 * func(x);
        }
    }
    return s * h / 3;
}

// 计算曲线长度
double lenth_calculate(cv::Point3f last_location, cv::Point3f new_location) {

    // 定义积分区间 [a, b]
    double a = last_location.y;
    double b = new_location.y;

    // 定义积分精度（分割数）
    int n = 500;

    // 弧长积分函数
    auto arcLengthFunc = [&](double x) {
        double derivative = poly.derivative(x);
        return sqrt(1 + derivative * derivative);
    };

    // 计算曲线长度
    double length = simpsonIntegration(arcLengthFunc, a, b, n);
    return length;
}


double speed_calculate(queue<pair<cv::Point3f,clock_t>> location) {
    double speed = 0;
    std::queue<pair<cv::Point3f,clock_t>> point = location;
    double time = (point.back().second - point.front().second) / CLOCKS_PER_SEC;
    while(point.size() > 1) {
        cv::Point3f last_location = point.front().first;
        point.pop();
        cv::Point3f new_location = point.front().first;
        speed += lenth_calculate(last_location, new_location);
    }
    speed /= time;
    return speed;
}
vector<cv::Point2f> location_predict(std::queue<pair<cv::Point3f,clock_t>>location ,cv::Point3f &possible_point) {
    if(location.size()>2)
    {
        vector<cv::Point3f> point;
        vector<pair<cv::Point3f,clock_t>> location_temp;
        while(location.size() > 1) {
            location_temp.push_back(location.front());
            location.pop();
        }
        for(auto i:location_temp)
        {
            point.push_back(i.first);
        }
        polynomialRegression(point, 2);
    }
    else return {};
    double speed = speed_calculate(location);
    double time = (location.back().second - location.front().second) / CLOCKS_PER_SEC;
    double length = speed * time;
    bool direction = location.back().first.y - location.front().first.y > 0;
    //图像中画出预测轨迹
    vector<cv::Point2f> predict_location;
    for(int i =0 ;i < Data::map.rows*10; i++)
    {
        if(poly(i) < 0||poly(i) >= Data::map.cols*10)
        continue;
        cv::Point2f point = cv::Point2f(poly(i)/10,Data::map.rows - i/10);
        predict_location.push_back(point);
    }
    if(direction)
    {
        int y = 0;
        while(lenth_calculate(location.front().first,cv::Point3f(poly(y),y,0)) < length&& y < 2800 && 0 < poly(y)&& poly(y) < 14000)
        {
            possible_point = cv::Point3f(poly(y),y,0);
            y+=10;
        }
    }
    else
    {
        int y = 0;
       while(lenth_calculate(location.front().first,cv::Point3f(poly(y),y,0)) < length&& y > 0 && 0 < poly(y)&& poly(y) < 14000)
        {
            possible_point = cv::Point3f(poly(y),y,0);
            y-=10;
        }
    }
    if(Data::is_beoccluded[int(possible_point.x/10)][int(possible_point.y/10)])
    {
        return predict_location;
    }
    return {};
}
