#ifndef _CIRCLE_POSE_ESTIMATION_H_
#define _CIRCLE_POSE_ESTIMATION_H_

#include "EDLib.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

#define NO_ROOTS 0
#define ONE_ROOTS 1
#define TWO_ROOTS 2
#define THREE_ROOTS 3

#define thera 5
#define minimum_area 1500

struct stereo_camera
{
    Eigen::Matrix<double, 3, 3> K_left, K_right;
    const double baseline;
    const double bf; // bf = baseline * fx
    const double width, height;

    stereo_camera() : width(848), height(480), baseline(0.095), bf(40.56), K_left(), K_right(){};
};

//描述圆锥体的参数:a*x^2+b*y^2+c*z^2+2*f*y*z+2*g*x*z+2*h*x*y+2*u*x+2*v*y+2wz+d=0
struct cone
{
    double a, b, c, f, g, h, u, v, w, d;
    cone(double a_, double b_, double c_, double f_, double g_, double h_,
         double w_, double u_, double v_, double d_)
    {
        a = a_;
        b = b_;
        c = c_;
        f = f_;
        g = g_;
        h = h_;
        u = u_;
        v = v_;
        w = w_;
        d = d_;
    }
};

/*
This struct describe a cubic polynomial in real number field, which contains functions of equation solving and root sorting
这个结构体用来描述一个在实数域上的三次方程，并且包含解方程和根排序的函数

A*x^3+B*x^2+C*x+D=0
*/
struct cubic_polynomial
{
    std::vector<double> roots;
    double coefficients[4];
    int style;
    int numberOfroots;

    cubic_polynomial() : coefficients(), roots(), numberOfroots(){};

    cubic_polynomial(double A, double B, double C, double D)
    {
        coefficients[0] = A;
        coefficients[1] = B;
        coefficients[2] = C;
        coefficients[3] = D;
    }
    // this function return roots of this cubic equation
    //解三次方程函数
    std::vector<double> solve();

    //根排序
    // int sort()
    // {
    //     return numberOfroots;
    // }
};

class circle_pose
{
public:
    mEllipse ellipse_left_raw, ellipse_right_raw;                          //初次从图像上提取的椭圆方程，坐标原点在左上
    EllipseEquation normalized_elli_equa_left, normalized_elli_equa_right; // H=K'CK
// public:
    bool is_valid; // flag 用以表示是否成功提取圆位姿
    double level_angle;
    Eigen::Vector3d position, normal; //位置与法向量
    cv::Mat img_left, img_right;
    cv::Point2d central_left_raw, central_right_raw; //为原像素平面上的椭圆中心
    stereo_camera cam;
    Eigen::Matrix<double, 4, 4> transition; //用以与ros通信
    circle_pose(){}
    // utility functions
    bool getEllipse(const cv::Mat &img, mEllipse &ellipse_raw);                                                 //获取椭圆
    void normalize(const mEllipse &ellipse_raw, EllipseEquation &normalized_elli_equa);                         //将椭圆方程映射到归一化平面上
    Eigen::Vector3d getNormal(EllipseEquation &normal_elli_equa_left, EllipseEquation &normal_elli_equa_right); //获得圆法向量
    Eigen::Vector3d getPosition(const cv::Point2d &central_left, const cv::Point2d &central_right);             //获得圆位置
    void getAngle() { level_angle = -atan(normal[0] / normal[2]); };
    void debug(); //调用该函数打印信息用以debug

    //执行此函数更新一次位姿
    bool processOnce(); //这个函数通过调用上面的函数来更新该类中的各种成员，来获得圆的位姿
};
#endif