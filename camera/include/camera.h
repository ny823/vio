#include <string>
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
class camera
{
public:
    cv::Mat frame;
    cv::Mat frame_undistort;
    cv::Mat mask;
    int height;
    int width;
    std::string cam_name;
    std::string img_topic;
    Eigen::Matrix<double, 3, 3> intrinsic;
    Eigen::Matrix<double, 4, 4> transfer;
    double distortion[4];
    camera(const std::string &name)
    {
        this->cam_name = name;
    }
    camera(const std::string &name, const Eigen::Matrix<double, 3, 3> &intri, const Eigen::Matrix<double, 4, 4> &tran)
    {
        this->cam_name = name;
        this->intrinsic = intri;
        this->transfer = tran;
    }
    virtual bool LoadFromYAML(const std::string &filename);
    virtual bool DeDistortion();
};