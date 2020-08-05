#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace roboiitk::CentreDetect {

class CentreDetect {
  private:
    ros::Publisher box_pub_, box_centre_pub;
    ros::Publisher image_pub_preprocess_hsv_;

    ros::Subscriber image_sub_;

    cv::Mat src_, processed_frame_, result, result1, img_hsv;

    geometry_msgs::Point center;

    int low_H = 0, low_S = 0, low_V = 225, high_H = 255, high_S = 30, high_V = 255;
    double distance_ = 0.0;

  public:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run(ros::NodeHandle& nh_private);

    cv::Mat preprocess(cv::Mat& img);
    void findCentre(cv::Mat& frame);
};

}  // namespace roboiitk::CentreDetect
