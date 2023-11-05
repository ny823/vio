#include "circle_pose_estimation.h"
std::vector<double> cubic_polynomial::solve()
{
    double A = this->coefficients[1] * this->coefficients[1] - 3 * this->coefficients[0] * this->coefficients[2];
    double B = this->coefficients[1] * this->coefficients[2] - 9 * this->coefficients[0] * this->coefficients[3];
    double C = this->coefficients[2] * this->coefficients[2] - 3 * this->coefficients[1] * this->coefficients[3];
    double f = B * B - 4 * A * C;
    double i_value;
    double Y1, Y2;
    if ((fabs(A) > 1e-6 || fabs(B) > 1e-6) && f < -1e-6) //公式4,三异实根
    {
        this->style = 3;
        double T = (2 * A * this->coefficients[1] - 3 * this->coefficients[0] * B) / (2 * A * sqrt(A));
        double S = acos(T);
        this->roots.push_back((-this->coefficients[1] - 2 * sqrt(A) * cos(S / 3)) / (3 * this->coefficients[0]));
        this->roots.push_back((-this->coefficients[1] + sqrt(A) * (cos(S / 3) + sqrt(3.0) * sin(S / 3))) / (3 * this->coefficients[0]));
        this->roots.push_back((-this->coefficients[1] + sqrt(A) * (cos(S / 3) - sqrt(3.0) * sin(S / 3))) / (3 * this->coefficients[0]));
    }
    if (fabs(A) < 1e-6 && fabs(B) < 1e-6) //公式1，三重根
    {
        this->roots.push_back(-this->coefficients[1] / (3 * this->coefficients[0]));
        this->roots.push_back(-this->coefficients[1] / (3 * this->coefficients[0]));
        this->roots.push_back(-this->coefficients[1] / (3 * this->coefficients[0]));
        this->style = 1;
    }
    if ((fabs(A) > 1e-6 || fabs(B) > 1e-6) && fabs(f) < 1e-6) //公式3，双重根
    {
        double K = B / A;
        this->roots.push_back(-this->coefficients[1] / this->coefficients[0] + K);
        this->roots.push_back(-K / 2);
        this->roots.push_back(-K / 2);
        this->style = 2;
    }
    if ((fabs(A) > 1e-6 || fabs(B) > 1e-6) && f > 1e-6) //公式2，虚根
    {
        Y1 = A * this->coefficients[1] + 3 * this->coefficients[0] * (-B + sqrt(f)) / 2;
        Y2 = A * this->coefficients[1] + 3 * this->coefficients[0] * (-B - sqrt(f)) / 2;
        double Y1_value = (Y1 / fabs(Y1)) * pow((double)fabs(Y1), 1.0 / 3);
        double Y2_value = (Y2 / fabs(Y2)) * pow((double)fabs(Y2), 1.0 / 3);
        this->roots.push_back((-this->coefficients[1] - Y1_value - Y2_value) / (3 * this->coefficients[0])); //虚根我不要
        //虚根还是看看吧，如果虚根的i小于0.1，则判定为方程的一根吧。。。
        i_value = sqrt(3.0) / 2 * (Y1_value - Y2_value) / (3 * this->coefficients[0]);
        if (fabs(i_value) < 1e-1)
        {
            this->roots.push_back((-this->coefficients[1] + 0.5 * (Y1_value + Y2_value)) / (3 * this->coefficients[0]));
        }
        this->style = 0;
    }
    return this->roots;
}
bool circle_pose::getEllipse(const cv::Mat &img, mEllipse &ellipse_raw) //获取椭圆
{
    ED testEDPF = ED(img, SOBEL_OPERATOR, 36, 0, 1, 10, 1.0, true); // apply ED algorithm
    EDCircles testEDCircles = EDCircles(testEDPF);

    // cv::Mat edgePFImage = testEDPF.getEdgeImage();
    // cv::imshow("Edge Image Parameter Free", edgePFImage);

    // Get circle information as [cx, cy, r]
    std::vector<mCircle> circles = testEDCircles.getCircles();
    // Get ellipse information as [cx, cy, a, b, theta]
    std::vector<mEllipse> ellipses = testEDCircles.getEllipses();
    //  out_img = testEDCircles.drawResult(true, ImageStyle::BOTH);
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
    ellipse_raw = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    bool isappeared;
    if (noellipse != 0 && nocircle != 0)
    {
        if (pow(circles[find_max_circle].r, 2) > ellipses[find_max_ellipses].axes.area())
        {
            if (pow(circles[find_max_circle].r, 2) > minimum_area)
            {
                cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
                ellipse_raw = mEllipse(circles[find_max_circle].center, r, 0);
                isappeared = true;
            }
            else
            {
                isappeared = false;
            }
        }
        else
        {
            if (ellipses[find_max_ellipses].axes.area() > minimum_area)
            {
                isappeared = true;
                ellipse_raw = ellipses[find_max_ellipses];
            }
            else
            {
                isappeared = false;
            }
        }
    }
    if (nocircle != 0 && noellipse == 0)
    {
        if (pow(circles[find_max_circle].r, 2) > minimum_area)
        {
            cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
            ellipse_raw = mEllipse(circles[find_max_circle].center, r, 0);
            isappeared = true;
        }
        else
        {
            isappeared = false;
        }
    }
    if (nocircle == 0 && noellipse != 0)
    {
        if (ellipses[find_max_ellipses].axes.area() > minimum_area)
        {
            isappeared = true;
            ellipse_raw = ellipses[find_max_ellipses];
        }
        else
        {
            isappeared = false;
        }
    }
    if (nocircle == 0 && noellipse == 0)
    {
        isappeared = false;
        ellipse_raw = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    }
    return isappeared;
}
void circle_pose::normalize(const mEllipse &ellipse_raw, EllipseEquation &normalized_elli_equa)
{
    double degree = -ellipse_raw.theta;
    double A = pow(cos(degree), 2) / pow(ellipse_raw.axes.width, 2) + pow(sin(degree), 2) / pow(ellipse_raw.axes.height, 2);
    double B = -2 * cos(degree) * sin(degree) * (1 / pow(ellipse_raw.axes.width, 2) - 1 / pow(ellipse_raw.axes.height, 2));
    double C = pow(cos(degree), 2) / pow(ellipse_raw.axes.height, 2) + pow(sin(degree), 2) / pow(ellipse_raw.axes.width, 2);
    double D = -(2 * A * ellipse_raw.center.x + B * ellipse_raw.center.y);
    double E = -(2 * C * ellipse_raw.center.y + B * ellipse_raw.center.x);
    double F = A * pow(ellipse_raw.center.x, 2) + B * ellipse_raw.center.x * ellipse_raw.center.y + C * pow(ellipse_raw.center.y, 2) - 1;
    normalized_elli_equa.coeff[1] = A;
    normalized_elli_equa.coeff[2] = B;
    normalized_elli_equa.coeff[3] = C;
    normalized_elli_equa.coeff[4] = D;
    normalized_elli_equa.coeff[5] = E;
    normalized_elli_equa.coeff[6] = F;
}
std::vector<Eigen::Vector3d> eli_circle_two(EllipseEquation &elis, const Eigen::Matrix<double, 3, 3> &K, double e, bool &istrue)
{
    istrue = false;
    std::vector<Eigen::Vector3d> circle_dir;
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
    cubic_polynomial equ3 = cubic_polynomial(1, b, c, d);
    equ3.solve();
    // printf("style:%d,x1:%f,x2:%f,x3:%f\n", equ3.style,equ3.roots[0], equ3.roots[1], equ3.roots[2]);
    double x[3];
    x[0] = MAX(MAX(equ3.roots[0], equ3.roots[1]), equ3.roots[2]);
    x[2] = MIN(MIN(equ3.roots[0], equ3.roots[1]), equ3.roots[2]);
    x[1] = equ3.roots[0] + equ3.roots[1] + equ3.roots[2] - x[0] - x[2];
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
    if (equ3.style != 0)
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
        Eigen::Vector3d circle_dir_1_inho = Eigen::Vector3d(circle_dir_1(0, 0), circle_dir_1(1, 0), circle_dir_1(2, 0));
        Eigen::Vector3d circle_dir_2_inho = Eigen::Vector3d(circle_dir_2(0, 0), circle_dir_2(1, 0), circle_dir_2(2, 0));
        circle_dir.push_back(circle_dir_1_inho);
        circle_dir.push_back(circle_dir_2_inho);
        istrue = true;
    }
    return circle_dir;
}

double angle(Eigen::Vector3d &line_1, Eigen::Vector3d &line_2)
{
    double vec_two = abs(line_1[0] * line_2[0] + line_1[1] * line_2[1] + line_1[2] * line_2[2]);
    double abs_vec = sqrt((pow(line_1[0], 2) + pow(line_1[1], 2) + pow(line_1[2], 2)) * (pow(line_2[0], 2) + pow(line_2[1], 2) + pow(line_2[2], 2)));
    double angle = acos(vec_two / abs_vec);
    return angle * 180 / PI;
}

Eigen::Vector3d circle_pose::getNormal(EllipseEquation &normal_elli_equa_left, EllipseEquation &normal_elli_equa_right)
{
    std::vector<Eigen::Vector3d> eli_circle_left, eli_circle_right;
    bool istrue_left, istrue_right;
    eli_circle_left = eli_circle_two(normal_elli_equa_left, this->cam.K_left, 1, istrue_left);
    eli_circle_right = eli_circle_two(normal_elli_equa_right, this->cam.K_right, 1, istrue_right);
    Eigen::Vector3d dir;
    if (istrue_left && istrue_right)
    {
        this->is_valid = false;
        if (angle(eli_circle_left[0], eli_circle_right[0]) < thera)
        {
            this->is_valid = true;
            if (atan(abs(eli_circle_left[0][0] / eli_circle_left[0][2])) < atan(abs(eli_circle_right[0][0] / eli_circle_right[0][2])))
            {
                dir = eli_circle_left[0];
            }
            else
            {
                dir = eli_circle_right[0];
            }
        }
        if (angle(eli_circle_left[1], eli_circle_right[0]) < thera)
        {
            this->is_valid = true;
            if (atan(abs(eli_circle_left[1][0] / eli_circle_left[1][2])) < atan(abs(eli_circle_right[0][0] / eli_circle_right[0][2])))
            {
                dir = eli_circle_left[1];
            }
            else
            {
                dir = eli_circle_right[0];
            }
        }
        if (angle(eli_circle_left[0], eli_circle_right[1]) < thera)
        {
            this->is_valid = true;
            if (atan(abs(eli_circle_left[0][0] / eli_circle_left[0][2])) < atan(abs(eli_circle_right[1][0] / eli_circle_right[1][2])))
            {
                dir = eli_circle_left[0];
            }
            else
            {
                dir = eli_circle_right[1];
            }
        }
        if (angle(eli_circle_left[1], eli_circle_right[1]) < thera)
        {
            this->is_valid = true;
            if (atan(abs(eli_circle_left[1][0] / eli_circle_left[1][2])) < atan(abs(eli_circle_right[1][0] / eli_circle_right[1][2])))
            {
                dir = eli_circle_left[1];
            }
            else
            {
                dir = eli_circle_right[1];
            }
        }
    }
    // std::cout << "dir:" << dir[0] << "\t" << dir[1] << "\t" << dir[2] << std::endl;
    return dir;
}

Eigen::Vector3d circle_pose::getPosition(const cv::Point2d &central_left, const cv::Point2d &central_right)
{
    double dif = central_left.x - central_right.x;
    double depth = 40.56 / dif;
    Eigen::Matrix<double, 3, 1> normalized_left;
    Eigen::Matrix<double, 3, 1> normalized_right;
    normalized_left << central_left.x, central_left.y, 1;
    normalized_right << central_right.x, central_right.y, 1;
    Eigen::Matrix<double, 3, 1> new_left = this->cam.K_left.inverse() * normalized_left * depth;
    Eigen::Matrix<double, 3, 1> new_right = this->cam.K_right.inverse() * normalized_right * depth;
    Eigen::Vector3d elg_center = Eigen::Vector3d((new_left[0] + new_right[0]) / 2, (new_left[1] + new_right[1]) / 2, (new_left[2] + new_right[2]) / 2);
    // std::cout<<"elg_center:"<<elg_center[0]<<"\t"<<elg_center[1]<<"\t"<<elg_center[2]<<std::endl;
    return elg_center;
}

void circle_pose::debug()
{
    std::cout << "**************************************************************************" << std::endl;
    if (!this->is_valid)
    {
        std::cout << "失败！！" << std::endl;
    }
    else
    {
        std::cout << "成功" << std::endl;
        std::cout << "normalized_elli_equa_left:";
        std::cout << normalized_elli_equa_left.A() << "\t"
                  << normalized_elli_equa_left.B() << "\t"
                  << normalized_elli_equa_left.C() << "\t"
                  << normalized_elli_equa_left.D() << "\t"
                  << normalized_elli_equa_left.E() << "\t"
                  << normalized_elli_equa_left.F() << "\t" << std::endl;
        std::cout << "normalized_elli_equa_right:";
        std::cout << normalized_elli_equa_right.A() << "\t"
                  << normalized_elli_equa_right.B() << "\t"
                  << normalized_elli_equa_right.C() << "\t"
                  << normalized_elli_equa_right.D() << "\t"
                  << normalized_elli_equa_right.E() << "\t"
                  << normalized_elli_equa_right.F() << "\t" << std::endl;
        std::cout << "elg_normal:" << this->normal[0] << "\t" << this->normal[1] << "\t" << this->normal[2] << std::endl;
        std::cout << "elg_center:" << this->position[0] << "\t" << this->position[1] << "\t" << this->position[2] << std::endl;
        std::cout << "level_angle:" << this->level_angle << std::endl;
    }
    std::cout << "**************************************************************************" << std::endl;
}

bool circle_pose::processOnce()
{
    this->getEllipse(this->img_left, this->ellipse_left_raw);
    this->getEllipse(this->img_right, this->ellipse_right_raw);
    this->normalize(this->ellipse_left_raw, this->normalized_elli_equa_left);
    this->normalize(this->ellipse_right_raw, this->normalized_elli_equa_right);
    this->normal = this->getNormal(this->normalized_elli_equa_left, this->normalized_elli_equa_right);
    this->central_left_raw = this->ellipse_left_raw.center;
    this->central_right_raw = this->ellipse_right_raw.center;
    this->position = this->getPosition(this->central_left_raw, this->central_right_raw);
    this->getAngle();
    return this->is_valid;
}