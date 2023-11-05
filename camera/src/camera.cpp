#include <yaml-cpp/yaml.h>
#include "../include/camera.h"
#include <iostream>
#include <math.h>


bool camera::LoadFromYAML(const std::string &filename)
{
    YAML::Node data = YAML::LoadFile(filename);
    if (!data)
    {
        std::cerr << "Could not load camera from file: " << filename << std::endl;
        return false;
    }
    data = data[cam_name];
    if (!data)
    {
        std::cerr << "Camera with name '" << cam_name << "' does not exist in "
                  << "file: " << filename << std::endl;
        return false;
    }
    if (data["topic"].IsDefined())
    {
        img_topic = data["topic"].as<std::string>();
    }
    else
    {
        std::cerr << "parameter topic does not exist" << std::endl;
        return false;
    }
    if (data["height"].IsDefined())
    {
        height = (int)data["height"].as<double>();
    }
    else
    {
        std::cerr << "parameter height does not exist" << std::endl;
        return false;
    }
    if (data["width"].IsDefined())
    {
        height = (int)data["width"].as<double>();
    }
    else
    {
        std::cerr << "parameter width does not exist" << std::endl;
        return false;
    }
    if (data["K"].IsDefined())
    {
        double K[4];
        for (int i = 0; i < (int)data["K"].size(); i++)
        {
            K[i] = data["K"][i].as<double>();
        }
        intrinsic << K[0], 0, K[2], 0, K[1], K[3], 0, 0, 1;
    }
    else
    {
        std::cerr << "parameter K does not exist" << std::endl;
        return false;
    }
    if (data["distortion"].IsDefined())
    {
        for (int i = 0; i < (int)data["distortion"].size(); i++)
        {
            distortion[i] = data["distortion"][i].as<double>();
        }
    }
    else
    {
        std::cerr << "parameter distortion does not exist" << std::endl;
        return false;
    }
    if (data["T_imu_cam"].IsDefined())
    {
        // get K

        double trans[16];

        for (int i = 0; i < (int)data["T_imu_cam"].size(); i++)
        {
            trans[i] = data["T_imu_cam"][i].as<double>();
        }

        transfer << trans[0], trans[1], trans[2], trans[3],
            trans[4], trans[5], trans[6], trans[7],
            trans[8], trans[9], trans[10], trans[11],
            trans[12], trans[13], trans[14], trans[15];
    }
    else
    {
        std::cerr << "parameter T_imu_cam does not exist" << std::endl;
        return false;
    }
    return true;
}

bool camera::DeDistortion()
{
    if (frame.type() != CV_8UC1)
    {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    }
    frame_undistort = cv::Mat(frame.rows, frame.cols, CV_8UC1);
    double fx, fy, cx, cy, k1, k2, p1, p2;
    fx = intrinsic(0, 0);
    fy = intrinsic(1, 1);
    cx = intrinsic(0, 2);
    cy = intrinsic(1, 2);
    k1 = distortion[0];
    k2 = distortion[1];
    p1 = distortion[2];
    p2 = distortion[3];
    for (int v = 0; v < frame.rows; v++)
    {
        for (int u = 0; u < frame.cols; u++)
        {
            double u_distorted = 0, v_distorted = 0;

            double x1, y1, x2, y2;
            // normalized image coordinates
            x1 = (u - cx) / fx;
            y1 = (v - cy) / fy;
            double r2;
            r2 = pow(x1, 2) + pow(y1, 2);
            x2 = x1 * (1 + k1 * r2 + k2 * pow(r2, 2)) + 2 * p1 * x1 * y1 + p2 * (r2 + 2 * x1 * x1);
            y2 = y1 * (1 + k1 * r2 + k2 * pow(r2, 2)) + p1 * (r2 + 2 * y1 * y1) + 2 * p2 * x1 * y1;
            u_distorted = fx * x2 + cx;
            v_distorted = fy * y2 + cy;

            if (u_distorted >= 0 && v_distorted >= 0 && (int)round(u_distorted) < frame.cols && (int)round(v_distorted) < frame.rows)
            {
                frame_undistort.at<uchar>(v, u) = frame.at<uchar>((int)round(v_distorted), (int)round(u_distorted));
            }
            else
            {
                frame_undistort.at<uchar>(v, u) = 0;
                // To do generate mask
            }
        }
    }
    return true;
}