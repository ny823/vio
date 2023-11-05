#include "EDLib.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
// struct vec_ax
// {
//     double x;
//     double y;
//     double z;
//     vec_ax(double _x, double _y, double _z)
//     {
//         x = _x;
//         y = _y;
//         z = _z;
//     }
// };

/************************************************************************/
/* 盛金公式求解三次方程的解
   德尔塔f=B^2-4AC
       这里只要了实根，虚根需要自己再整理下拿出来
*/
/************************************************************************/



void ShengJin(double a, double b, double c, double d, std::vector<double> &X123, int &style)
{
    double A = b * b - 3 * a * c;
    double B = b * c - 9 * a * d;
    double C = c * c - 3 * b * d;
    double f = B * B - 4 * A * C;
    double i_value;
    double Y1, Y2;
    if ((fabs(A) > 1e-6 || fabs(B) > 1e-6) && f < -1e-6) //公式4
    {
        style = 4;
        double T = (2 * A * b - 3 * a * B) / (2 * A * sqrt(A));
        double S = acos(T);
        X123.push_back((-b - 2 * sqrt(A) * cos(S / 3)) / (3 * a));
        X123.push_back((-b + sqrt(A) * (cos(S / 3) + sqrt(3.0) * sin(S / 3))) / (3 * a));
        X123.push_back((-b + sqrt(A) * (cos(S / 3) - sqrt(3.0) * sin(S / 3))) / (3 * a));
    }
    if (fabs(A) < 1e-6 && fabs(B) < 1e-6) //公式1
    {
        X123.push_back(-b / (3 * a));
        X123.push_back(-b / (3 * a));
        X123.push_back(-b / (3 * a));
        style = 1;
    }
    if ((fabs(A) > 1e-6 || fabs(B) > 1e-6) && fabs(f) < 1e-6) //公式3
    {
        double K = B / A;
        X123.push_back(-b / a + K);
        X123.push_back(-K / 2);
        X123.push_back(-K / 2);
        style = 3;
    }
    if ((fabs(A) > 1e-6 || fabs(B) > 1e-6) && f > 1e-6) //公式2
    {
        Y1 = A * b + 3 * a * (-B + sqrt(f)) / 2;
        Y2 = A * b + 3 * a * (-B - sqrt(f)) / 2;
        double Y1_value = (Y1 / fabs(Y1)) * pow((double)fabs(Y1), 1.0 / 3);
        double Y2_value = (Y2 / fabs(Y2)) * pow((double)fabs(Y2), 1.0 / 3);
        X123.push_back((-b - Y1_value - Y2_value) / (3 * a)); //虚根我不要
        //虚根还是看看吧，如果虚根的i小于0.1，则判定为方程的一根吧。。。
        i_value = sqrt(3.0) / 2 * (Y1_value - Y2_value) / (3 * a);
        if (fabs(i_value) < 1e-1)
        {
            X123.push_back((-b + 0.5 * (Y1_value + Y2_value)) / (3 * a));
        }
        style = 2;
    }
    // std::cout<<"解:"<<X123[0]<<"\t"<<X123[1]<<"\t"<<X123[2]<<"\t"<<std::endl;
}
//描述圆锥体的参数:a*x^2+b*y^2+c*z^2+2*f*y*z+2*g*x*z+2*h*x*y+2*u*x+2*v*y+2wz+d=0
struct cone
{
    double a;
    double b;
    double c;
    double f;
    double g;
    double h;
    double u;
    double v;
    double w;
    double d;
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
const int area = 1500; //识别的椭圆在图片上的最小面积
// const int s = 300;//

//获取图片中的椭圆信息
mEllipse get_elp(cv::Mat &in_Img, bool &isappeared)
{
    ED testEDPF = ED(in_Img, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
    EDCircles testEDCircles = EDCircles(testEDPF);

    // cv::Mat edgePFImage = testEDPF.getEdgeImage();
    // cv::imshow("Edge Image Parameter Free", edgePFImage);

    // Get circle information as [cx, cy, r]
    std::vector<mCircle> circles = testEDCircles.getCircles();
    // Get ellipse information as [cx, cy, a, b, theta]
    std::vector<mEllipse> ellipses = testEDCircles.getEllipses();
    // out_img = testEDCircles.drawResult(true, ImageStyle::BOTH);
    int nocircle = circles.capacity();
    int noellipse = ellipses.capacity();
    int find_max_circle = 0;
    int find_max_ellipses = 0;
    if (nocircle != 0)
    {
        for (int i = 1; i < nocircle; i++)
        {
            if (circles[find_max_circle].r < circles[i].r)
                find_max_circle = i;
        }
    }
    if (noellipse != 0)
    {
        for (int i = 1; i < find_max_ellipses; i++)
        {
            if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
                find_max_ellipses = i;
        }
    }
    mEllipse c = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    if (noellipse != 0 && nocircle != 0)
    {
        if (pow(circles[find_max_circle].r, 2) > ellipses[find_max_ellipses].axes.area())
        {
            if (pow(circles[find_max_circle].r, 2) > area)
            {
                cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
                c = mEllipse(circles[find_max_circle].center, r, 0);
                isappeared = true;
            }
            else
            {
                isappeared = false;
            }
        }
        else
        {
            if (ellipses[find_max_ellipses].axes.area() > area)
            {
                isappeared = true;
                c = ellipses[find_max_ellipses];
            }
            else
            {
                isappeared = false;
            }
        }
    }
    if (nocircle != 0 && noellipse == 0)
    {
        if (pow(circles[find_max_circle].r, 2) > area)
        {
            cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
            c = mEllipse(circles[find_max_circle].center, r, 0);
            isappeared = true;
        }
        else
        {
            isappeared = false;
        }
    }
    if (nocircle == 0 && noellipse != 0)
    {
        if (ellipses[find_max_ellipses].axes.area() > area)
        {
            isappeared = true;
            c = ellipses[find_max_ellipses];
        }
        else
        {
            isappeared = false;
        }
    }
    if (nocircle == 0 && noellipse == 0)
    {
        isappeared = false;
        c = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    }
    return c;
}

//将椭圆转换为普通方程
EllipseEquation turn_mEllipse(mEllipse mellipse)
{
    double degree = -mellipse.theta;
    double A = pow(cos(degree), 2) / pow(mellipse.axes.width, 2) + pow(sin(degree), 2) / pow(mellipse.axes.height, 2);
    double B = -2 * cos(degree) * sin(degree) * (1 / pow(mellipse.axes.width, 2) - 1 / pow(mellipse.axes.height, 2));
    double C = pow(cos(degree), 2) / pow(mellipse.axes.height, 2) + pow(sin(degree), 2) / pow(mellipse.axes.width, 2);
    double D = -(2 * A * mellipse.center.x + B * mellipse.center.y);
    double E = -(2 * C * mellipse.center.y + B * mellipse.center.x);
    double F = A * pow(mellipse.center.x, 2) + B * mellipse.center.x * mellipse.center.y + C * pow(mellipse.center.y, 2) - 1;
    EllipseEquation eli;
    eli.coeff[1] = A;
    eli.coeff[2] = B;
    eli.coeff[3] = C;
    eli.coeff[4] = D;
    eli.coeff[5] = E;
    eli.coeff[6] = F;
    // std::cout << A << "\t" << B << "\t" << C << "\t" << D << "\t" << E << "\t" << F << "\t" << std::endl;
    return eli;
}

//计算两向量之间的夹角
double angle(const cv::Point3d &line_1, const cv::Point3d &line_2)
{
    double vec_two = abs(line_1.x * line_2.x + line_1.y * line_2.y + line_1.z * line_2.z);
    double abs_vec = sqrt((pow(line_1.x, 2) + pow(line_1.y, 2) + pow(line_1.z, 2)) * (pow(line_2.x, 2) + pow(line_2.y, 2) + pow(line_2.z, 2)));
    double angle = acos(vec_two / abs_vec);
    return angle * 180 / PI;
}
/************************************************************************/
/* 由图像的椭圆得到现实的圆的方向，elis为像素坐标下的椭圆普通方程，K为相机的内参方程，e为焦距,left_e为左相机中的椭圆e
平面方程为：lx+my+nz=1
 */
/************************************************************************/
std::vector<cv::Point3d> eli_circle_two(EllipseEquation &elis, const Eigen::Matrix<double, 3, 3> &K, double e, bool &istrue)
{
    istrue = false;
    std::vector<cv::Point3d> circle_dir;
    Eigen::Vector4d circle_dir_1, circle_dir_2, circle_dir_1_, circle_dir_2_;
    Eigen::Matrix<double, 4, 4> rotate;
    Eigen::Matrix<double, 3, 3> elg;
    Eigen::Matrix<double, 3, 3> elg_K;
    elg << elis.A(), elis.B(), elis.D() / 2, elis.B(), elis.C(),
        elis.E() / 2, elis.D() / 2, elis.E() / 2, elis.F();
    elg_K = K.transpose() * elg * K;
    // std::cout<<"elg_K"<<elg_K<<std::endl;
    cone cone_e = cone(pow(e, 2) * elg_K(0, 0), pow(e, 2) * elg_K(1, 1), elg_K(2, 2),
                       e * elg_K(1, 2), e * elg_K(0, 2), pow(e, 2) * elg_K(0, 1),
                       pow(e, 2) * elg_K(0, 2), pow(e, 2) * elg_K(1, 2), e * elg_K(2, 2),
                       pow(e, 2) * elg_K(2, 2));
    double c = cone_e.b * cone_e.c + cone_e.b * cone_e.a + cone_e.a * cone_e.c - pow(cone_e.f, 2) - pow(cone_e.g, 2) - pow(cone_e.h, 2);
    double d = -(cone_e.b * cone_e.c * cone_e.a + 2 * cone_e.f * cone_e.g * cone_e.h - cone_e.b * cone_e.g * cone_e.g - cone_e.h * cone_e.c * cone_e.h - cone_e.f * cone_e.f * cone_e.a);
    double b = -(cone_e.a + cone_e.b + cone_e.c);
    int style = 0;
    std::vector<double> X123;
    ShengJin(1, b, c, d, X123, style);
    double x[3];
    x[0] = MAX(MAX(X123[0], X123[1]), X123[2]);
    x[2] = MIN(MIN(X123[0], X123[1]), X123[2]);
    x[1] = X123[0] + X123[1] + X123[2] - x[0] - x[2];
    double m[3], n[3], l[3], t[3];
    double l_1, l_2, n_, m_;
    for (int i = 0; i < 3; i++)
    {
        t[0] = (cone_e.b - x[i]) * cone_e.g - cone_e.f * cone_e.h;
        t[1] = (cone_e.a - x[i]) * cone_e.f - cone_e.g * cone_e.h;
        t[2] = -(cone_e.a - x[i]) * (t[0] / t[1]) / cone_e.g - cone_e.h / cone_e.g;
        m[i] = 1 / sqrtf(1 + pow(t[0] / t[1], 2) + pow(t[2], 2));
        l[i] = (t[0] / t[1]) * m[i];
        n[i] = t[2] * m[i];
    }
    rotate << l[0], l[1], l[2], 0, m[0], m[1], m[2], 0, n[0], n[1], n[2], 0, 0, 0, 0, 1;
    std::cout << style << std::endl;
    if (style != 2)
    {
        if (fabs(x[0] - x[1]) < 1e-6)
        {
            n_ = 1.0;
            m_ = 0.0;
            l_1 = l_2 = 0;
        }
        if (fabs(x[0] - x[1]) > 1e-6)
        {
            l_1 = sqrt((x[0] - x[1]) / (x[1] - x[2]));
            l_2 = -l_1;
            m_ = 0.0;
            n_ = sqrt((x[1] - x[2]) / (x[0] - x[2]));
        }
        circle_dir_1_ << l_1, m_, n_, 1;
        circle_dir_2_ << l_2, m_, n_, 1;
        circle_dir_1 = rotate * circle_dir_1_;
        circle_dir_2 = rotate * circle_dir_2_;
        cv::Point3d circle_dir_1_inho = cv::Point3d(circle_dir_1(0, 0), circle_dir_1(1, 0), circle_dir_1(2, 0));
        cv::Point3d circle_dir_2_inho = cv::Point3d(circle_dir_2(0, 0), circle_dir_2(1, 0), circle_dir_2(2, 0));
        circle_dir.push_back(circle_dir_1_inho);
        circle_dir.push_back(circle_dir_2_inho);
        istrue = true;
        std::cout << "line_1:" << circle_dir_1.transpose() << std::endl;
        std::cout << "line_2:" << circle_dir_2.transpose() << std::endl;
        std::cout << std::endl;
    }
    return circle_dir;
}
double qumo(cv::Point3d a)
{
    return sqrt(pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2));
}

cv::Point3d true_dir(const std::vector<cv::Point3d> &circle_dir_left, const std::vector<cv::Point3d> &circle_dir_right, double thera, bool &istrue)
{
    istrue = false;
    cv::Point3d dir;
    if (angle(circle_dir_left[0], circle_dir_right[0]) < thera)
    {
        istrue = true;
        if (atan(abs(circle_dir_left[0].x/circle_dir_left[0].z))<atan(abs(circle_dir_right[0].x/circle_dir_right[0].z)))
        {
            dir = circle_dir_left[0];
        }
        else
        {
            dir = circle_dir_right[0];
        }
    }
    if (angle(circle_dir_left[1], circle_dir_right[0]) < thera)
    {
        istrue = true;
        if (atan(abs(circle_dir_left[1].x/circle_dir_left[1].z))<atan(abs(circle_dir_right[0].x/circle_dir_right[0].z)))
        {
            dir = circle_dir_left[1];
        }
        else
        {
            dir = circle_dir_right[0];
        }
    }
    if (angle(circle_dir_left[0], circle_dir_right[1]) < thera)
    {
        istrue = true;
        if (atan(abs(circle_dir_left[0].x/circle_dir_left[0].z))<atan(abs(circle_dir_right[1].x/circle_dir_right[1].z)))
        {
            dir = circle_dir_left[0];
        }
        else
        {
            dir = circle_dir_right[1];
        }
    }
    if (angle(circle_dir_left[1], circle_dir_right[1]) < thera)
    {
        istrue = true;
        if (atan(abs(circle_dir_left[1].x/circle_dir_left[1].z))<atan(abs(circle_dir_right[1].x/circle_dir_right[1].z)))
        {
            dir = circle_dir_left[1];
        }
        else
        {
            dir = circle_dir_right[1];
        }
    }
    std::cout<<"dir:"<<dir.x<<"\t"<<dir.y<<"\t"<<dir.z<<std::endl;
    return dir;
}
cv::Point3d elg_location(cv::Point2d &elg1, cv::Point2d &elg2)
{
    Eigen::Matrix<double, 3, 3> K_left, K_right;
    K_left << 427.2486928456254, 0, 425.06874497136005, 0, 427.74610622571066, 236.16152744508838, 0, 0, 1;
    K_right << 425.81737868037914, 0, 424.41982264160583, 0, 426.2683663190262, 235.4487746155112, 0, 0, 1;
    double dif = elg1.x - elg2.x;
    double depth = 40.56 / dif;
    Eigen::Matrix<double, 3, 1> normalized_left;
    Eigen::Matrix<double, 3, 1> normalized_right;
    normalized_left<<elg1.x, elg1.y, 1;
    normalized_right<<elg2.x, elg2.y, 1;
    Eigen::Matrix<double, 3, 1> new_left = K_left.inverse()*normalized_left*depth;
    Eigen::Matrix<double, 3, 1> new_right = K_right.inverse()*normalized_right*depth;
    cv::Point3d elg_center = cv::Point3d((new_left[0]+new_right[0])/2, (new_left[1]+new_right[1])/2, (new_left[2]+new_right[2])/2);
    std::cout<<"elg_center:"<<elg_center.x<<"\t"<<elg_center.y<<"\t"<<elg_center.z<<std::endl;
    return elg_center;
}

std::vector<cv::Point3d> zuanquan(cv::Mat &Img_left, cv::Mat &Img_right, const Eigen::Matrix<double, 3, 3> &K_left, const Eigen::Matrix<double, 3, 3> &K_right, bool &isapp)
{
    bool isappeared_left, isappeared_right;
    bool istrue_left, istrue_right, istrue;
    isapp = false;
    std::vector<cv::Point3d> eli_circle_left, eli_circle_right;
    std::vector<cv::Point3d> vec;
    mEllipse eli_left = get_elp(Img_left, isappeared_left);
    mEllipse eli_right = get_elp(Img_right, isappeared_right);
    if (isappeared_left && isappeared_right)
    {
        isapp = true;
        EllipseEquation eli_l_ell = turn_mEllipse(eli_left);
        EllipseEquation eli_r_ell = turn_mEllipse(eli_right);
        eli_circle_left = eli_circle_two(eli_l_ell, K_left, 1, istrue_left);
        eli_circle_right = eli_circle_two(eli_r_ell, K_right, 1, istrue_right);
        vec.push_back(true_dir(eli_circle_left, eli_circle_right, 5, istrue));
        vec.push_back(elg_location(eli_left.center, eli_right.center));
    }
    return vec;
}