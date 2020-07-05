#include <rover_aruco_detector/aruco_centre.hpp>

namespace roboiitk::CentreDetect{
    cv::Mat CentreDetect::preprocess(cv::Mat& img){
        cv::Mat kernel = cv::Mat::eye(7, 7, CV_8U);
        cv::cvtColor(img, img_hsv, CV_BGR2HSV);
        cv::inRange(img_hsv,cv::Scalar(low_H, low_S, low_V),cv::Scalar(high_H, high_S, high_V),result1);
        image_pub_preprocess_hsv_.publish((cv_bridge::CvImage(std_msgs::Header(), "mono8", result1).toImageMsg()));
        return result1;
    }
    void CentreDetect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private){
        nh_private.getParam("high_H",high_H);nh_private.getParam("high_S",high_S);nh_private.getParam("high_V",high_V);
        nh_private.getParam("low_H",low_H);nh_private.getParam("low_S",low_S);nh_private.getParam("low_V",low_V);
        image_sub_ = nh.subscribe("/rover/camera/image_raw", 1, &CentreDetect::imageCb, this);
        image_pub_preprocess_hsv_=nh.advertise<sensor_msgs::Image>("preprocessed_image_hsv", 1);
        box_pub_=nh.advertise<sensor_msgs::Image>("aruco_box", 1);
        box_centre_pub=nh.advertise<geometry_msgs::Point>("aruco_box_centre", 1);
    }
    void CentreDetect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
        catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what());}
        src_ = cv_ptr->image;
        ROS_ASSERT(src_.empty() != true);
        findCentre(src_);
    }
    void CentreDetect::findCentre(cv::Mat& src_){
        processed_frame_ = preprocess(src_);
        std::vector<std::vector<cv::Point>> list_contours;
        cv::Mat drawing;
        cv::findContours(processed_frame_, list_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> hull(list_contours.size());
        std::vector<cv::Point> corners;
        std::vector<std::vector<cv::Point>> list_corners;

        for (int i = 0; i < list_contours.size(); i++){
            cv::convexHull(cv::Mat(list_contours[i]), hull[i]);
            if(cv::contourArea(hull[i])<5)continue;
            cv::approxPolyDP(cv::Mat(hull[i]),corners,cv::arcLength(cv::Mat(hull[i]), true) * 0.05,true);
            list_corners.push_back(corners);     
            if (corners.size() == 4) {
                cv::drawContours(drawing, list_corners, 0, cv::Scalar(255, 0, 0),1, 8);
                distance_= -12;
                center.x=float((list_corners.at(0).at(0).x + list_corners.at(0).at(1).x + list_corners.at(0).at(2).x +list_corners.at(0).at(3).x) /4);
                center.y=float((list_corners.at(0).at(0).y + list_corners.at(0).at(1).y + list_corners.at(0).at(2).y +list_corners.at(0).at(3).y) /4);
                center.z = float(distance_);
                box_centre_pub.publish(center);
                ROS_INFO("Aruco Box Detected");
                box_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg());
            }
            list_corners.clear();
            corners.clear();
        }
    }

    void CentreDetect::run(ros::NodeHandle& nh_private){
        nh_private.getParam("high_H",high_H);nh_private.getParam("high_S",high_S);nh_private.getParam("high_V",high_V);
        nh_private.getParam("low_H",low_H);nh_private.getParam("low_S",low_S);nh_private.getParam("low_V",low_V);
    }   
} // roboiitk::CentreDetect
