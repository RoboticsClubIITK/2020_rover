#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace roboiitk::detect {
class box_detect {
    private:
	ros::Publisher image_pub;
	ros::Publisher image_pub_preprocess;
	ros::Subscriber image_sub;
	ros::Publisher box_centre_pub;
	bool publish_preprocess;
	bool publish_detected_box;
	double error_limit;
	int contour_perimeter_thresh;
	int kernel_size_;
	geometry_msgs::Point center;

    public:
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
	void run();
	cv::Mat preprocess(cv::Mat& img);
};
} // namespace roboiitk::detect