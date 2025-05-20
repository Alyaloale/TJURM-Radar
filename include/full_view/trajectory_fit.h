#ifndef RM2024_FULL_VIEW_TRAJECTORY_FIT_H_
#define RM2024_FULL_VIEW_TRAJECTORY_FIT_H_

#include "data_manager/param.h"
#include "data_manager/base.h"


// 定义多项式函数及其导数
class Polynomial {
public:
    // 构造函数，接收多项式系数，系数按升幂排列
   void update(std::vector<double> coeffs){
        coefficients.clear();
        for(int i = 0; i < coeffs.size(); i++){
            coefficients.push_back(coeffs[i]);
        }
    }

    // 计算多项式值
    double operator()(double x) const {
        double result = 0;
        for (int i = 0; i < coefficients.size(); i++) {
            result += coefficients[i] * pow(x, i);
        }
        return result;
    }

    // 计算多项式导数值
    double derivative(double x) const {
        double result = 0;
        for (int i = 1; i < coefficients.size(); i++) {
            result += i * coefficients[i] * pow(x, i - 1);
        }
        return result;
    }

public:
    std::vector<double> coefficients;
};
void polynomialRegression(std::vector<cv::Point3f> &location, int degree);//一元多项式回归函数
double simpsonIntegration(const std::function<double(double)>& func, double a, double b, int n);//数值积分函数
double lenth_calculate(cv::Point3f last_location, cv::Point3f new_location);//计算曲线长度
std::vector<cv::Point2f> location_predict(std::queue<std::pair<cv::Point3f,clock_t>>location ,cv::Point3f &possible_point);//预测位置
double speed_calculate(std::queue<std::pair<cv::Point3f,clock_t>> location);//计算速度



#endif