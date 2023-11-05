#include "ros/ros.h"
#include "../include/state_machine.hpp"
#include "../include/camera.h"
#include "../include/circle_pose_estimation.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "tf/transform_datatypes.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
namespace sml = boost::sml;
using namespace message_filters;
using namespace sml;

void image_callback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img, cv::Mat &img_1, cv::Mat &img_2)
{
    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::TYPE_8UC1);
    img_1 = cv_ptr1->image;
    cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::TYPE_8UC1);
    img_2 = cv_ptr2->image;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_fly");
    drone::dependencies d;
    ros::NodeHandle nh;
    camera left_cam = camera("cam0");
    camera right_cam = camera("cam1");
    left_cam.LoadFromYAML("/home/do/smldev_ws/src/camera/yaml/test.yaml");
    right_cam.LoadFromYAML("/home/do/smldev_ws/src/camera/yaml/test.yaml");
    message_filters::Subscriber<sensor_msgs::Image> sub_left_image(nh, left_cam.img_topic, 2000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> sub_left_image(nh, right_cam.img_topic, 2000, ros::TransportHints().tcpNoDelay());
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_left_image, sub_right_image);
    sync.registerCallback(boost::bind(&image_callback, _1, _2, &left_cam.frame, &right_cam.frame));
    ros::Rate rate(15);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    drone::TFtarget target;
    bool is_detected;
    geometry_msgs::Quaternion rotate;
    tf::Quaternion tf_rotate;
    std::vector<cv::Point3d> normal;
    stereo_camera camera;
    camera.K_left = left_cam.intrinsic;
    camera.K_right = right_cam.intrinsic;
    camera.height = left_cam.height;
    camera.width = right_cam.width;
    camera.baseline = 0.075;
    camera.bf = 40.56;
    circle_pose CROSS_O;
    CROSS_O.cam = camera;
    CROSS_O.img_left = &left_cam.frame;
    CROSS_O.img_right = &right_cam.frame;
    sm.process_event(drone::release{});
    target.SetTarget(0, 0, 0, 1, 0, 0, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }
    bool process = CROSS_O.processOnce();
    if (!process)
    {
        process = CROSS_O.processOnce();
    }
    target.SetPosition(CROSS_O.position(2) - 0.5, -CROSS_O.position(0) + 0.5, 1.1);
    geometry_msgs::Quaternion rotate = tf::createQuaternionMsgFromYaw(CROSS_O.level_angle);
    tf::Quaternion tf_rotate;
    tf::quaternionMsgToTF(rotate, tf_rotate);
    tf::Quaternion new_rotate = tf::Quaternion(0, 0, 0, 1) * tf_rotate;
    target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }
    return 0;
}