#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace roboiitk::ArucoDetect {

class ArucoDetect {
  private:
    ros::Publisher aruco_marker_pub_, aruco_marker_centre_pub_;
    ros::Subscriber image_sub_;

    cv::Mat src_, aruco_marker_drawing_;

    geometry_msgs::Point center_;
    double distance_ = 0.0;

  public:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run(ros::NodeHandle& nh_private);

    void findCentre(cv::Mat& frame);
};

}  // namespace roboiitk::ArucoDetect
