#include <rover/detect_box.hpp>

namespace roboiitk::detect {
cv::Mat box_detect::preprocess(cv::Mat& img) 
{
	ROS_ASSERT(img.empty() != true);

	cv::Mat result,img_hsv;
	cv::Mat kernel = cv::Mat::eye(kernel_size_, kernel_size_, CV_8U);

	cv::cvtColor(img, img_hsv, CV_BGR2HSV);
	cv::inRange(img_hsv,
	    cv::Scalar(75,100,20),
	    cv::Scalar(100,255,255),
	    result);
	cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    return result;
    }

void box_detect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	nh_private.getParam("publish_preprocess", publish_preprocess);
	nh_private.getParam("publish_detected_box", publish_detected_box);
	nh_private.getParam("error_limit", error_limit);
	nh_private.getParam("contour_perimeter_thresh", contour_perimeter_thresh);
	nh_private.getParam("kernel_size", kernel_size_);
	image_sub = nh.subscribe("/magnus/camera/image_raw", 1, &box_detect::imageCb, this);
	image_pub = nh.advertise<sensor_msgs::Image>("detected_box", 1);
	image_pub_preprocess = nh.advertise<sensor_msgs::Image>("preprocessed_image", 1);
	box_centre_pub = nh.advertise<geometry_msgs::Point>("platform_centre", 1);
}

void box_detect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
	cv::Mat frame;
	frame = cv_ptr->image;
	ROS_ASSERT(frame.empty() != true);

	std::vector<std::vector<cv::Point>> list_contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Mat processed_frame;
	processed_frame = preprocess(frame);
	if (publish_preprocess) {
		cv_bridge::CvImage preprocessed_img;
		preprocessed_img.encoding = sensor_msgs::image_encodings::MONO8;
		preprocessed_img.header.stamp = ros::Time::now();
		preprocessed_img.image = processed_frame;
		image_pub_preprocess.publish(preprocessed_img.toImageMsg());
	}

	cv::findContours(processed_frame, list_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<int>> hull;
  std::vector<std::vector<cv::Point>> hull1(list_contours.size());
  hull.resize(list_contours.size());

	cv::Mat drawing = cv::Mat::zeros(frame.size(), CV_8UC3);
	std::vector<cv::Point> corners;
  std::vector<std::vector<cv::Point>> list_corners;

	for (int i = 0; i < list_contours.size(); i++) {
        cv::convexHull(cv::Mat(list_contours[i]), hull[i]);
        cv::convexHull(cv::Mat(list_contours[i]), hull1[i]);
         
		if (std::fabs(cv::arcLength(cv::Mat(hull1[i]), true)) < contour_perimeter_thresh ) {
			continue;
		}

    cv::approxPolyDP(cv::Mat(hull1[i]),corners,cv::arcLength(cv::Mat(hull1[i]), true) * error_limit,true);
    list_corners.push_back(corners);     
    if (corners.size() == 4) {
      cv::drawContours(drawing, list_corners, 0, cv::Scalar(255, 0, 0),1, 8);
      center.x=(list_corners.at(0).at(0).x + list_corners.at(0).at(1).x + list_corners.at(0).at(2).x +list_corners.at(0).at(3).x) /4;
      center.y=(list_corners.at(0).at(0).y + list_corners.at(0).at(1).y + list_corners.at(0).at(2).y +list_corners.at(0).at(3).y) /4;
      center.z = 0.0;
      ROS_INFO("Box Detected");
    }
    list_corners.clear();
        corners.clear();  
  } 
	if (publish_detected_box) {
		cv_bridge::CvImage Detected_H;
		Detected_H.encoding = sensor_msgs::image_encodings::BGR8;
		Detected_H.header.stamp = ros::Time::now();
		Detected_H.image = drawing;
		image_pub.publish(Detected_H.toImageMsg());
	}
}

void box_detect::run() { box_centre_pub.publish(center); }
} // namespace roboiitk::detect