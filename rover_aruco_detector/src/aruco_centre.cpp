#include <rover_aruco_detector/aruco_centre.hpp>

namespace roboiitk::CentreDetect{
    cv::Mat CentreDetect::preprocess(cv::Mat& img){
        cv::Mat kernel = cv::Mat::eye(7, 7, CV_8U);
        cv::cvtColor(img, img_hsv, CV_BGR2HSV);
        //cv::inRange(img,cv::Scalar(200,200,200),cv::Scalar(255,255,255),result);
        cv::inRange(img_hsv,cv::Scalar(low_H, low_S, low_V),cv::Scalar(high_H, high_S, high_V),result1);
        /*cv::namedWindow("image_hsv");
        cv::imshow("image_hsv", result1);
        cv::waitKey(); **/
        cv_bridge::CvImage preprocessed_img_hsv;
        preprocessed_img_hsv.encoding = sensor_msgs::image_encodings::MONO8;
        preprocessed_img_hsv.header.stamp = ros::Time::now();
        preprocessed_img_hsv.image = result1;
        image_pub_preprocess_hsv.publish(preprocessed_img_hsv.toImageMsg());

        /*cv::morphologyEx(result1, result1, cv::MORPH_OPEN, kernel);
        cv::Mat element = getStructuringElement( cv::MORPH_CROSS,
                            cv::Size( 7, 7));
        cv::dilate(result1,result1, element);*/
        return result1;
    }
    void CentreDetect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private){
        nh_private.getParam("high_H",high_H);nh_private.getParam("high_S",high_S);nh_private.getParam("high_V",high_V);
        nh_private.getParam("low_H",low_H);nh_private.getParam("low_S",low_S);nh_private.getParam("low_V",low_V);
        image_sub_ = nh.subscribe("/rover/camera/image_raw", 1, &CentreDetect::imageCb, this);
        //image_pub_preprocess_=nh.advertise<sensor_msgs::Image>("preprocessed_image", 1);
        image_pub_preprocess_hsv=nh.advertise<sensor_msgs::Image>("preprocessed_image_hsv", 1);
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
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat drawing= cv::Mat::zeros(src_.size(), CV_8UC3);
        cv::findContours(processed_frame_, list_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> hull(list_contours.size());
        cv_bridge::CvImage box;
        std::vector<cv::Point> corners;
        std::vector<std::vector<cv::Point>> list_corners;

        for (int i = 0; i < list_contours.size(); i++){
            cv::convexHull(cv::Mat(list_contours[i]), hull[i]);
            if(cv::contourArea(hull[i])<5)continue;
            cv::approxPolyDP(cv::Mat(hull[i]),corners,cv::arcLength(cv::Mat(hull[i]), true) * 0.05,true);
            list_corners.push_back(corners);     
            if (corners.size() == 4) {
                cv::drawContours(drawing, list_corners, 0, cv::Scalar(255, 0, 0),1, 8);
                distance_=sqrt((list_corners.at(0).at(1).x-list_corners.at(0).at(0).x)*(list_corners.at(0).at(1).x-list_corners.at(0).at(0).x)+
                                (list_corners.at(0).at(1).y-list_corners.at(0).at(0).y)*(list_corners.at(0).at(1).y-list_corners.at(0).at(0).y));
                ROS_INFO("Distance_: %lf",distance_);
                distance_=scale_factor_/(distance_);
                center.x=(list_corners.at(0).at(0).x + list_corners.at(0).at(1).x + list_corners.at(0).at(2).x +list_corners.at(0).at(3).x) /4;
                center.y=(list_corners.at(0).at(0).y + list_corners.at(0).at(1).y + list_corners.at(0).at(2).y +list_corners.at(0).at(3).y) /4;
                center.z = distance_;
                box_centre_pub.publish(center);
                ROS_INFO("Aruco Box Detected");
                box.encoding = sensor_msgs::image_encodings::BGR8;
                box.header.stamp = ros::Time::now();
                box.image = drawing;
                box_pub_.publish(box.toImageMsg());
            }
            list_corners.clear();
            corners.clear();
            //cv::drawContours(drawing, hull, i, cv::Scalar(255, 0, 0),1, 8);
        }
    }

    void CentreDetect::run(ros::NodeHandle& nh_private){
        nh_private.getParam("high_H",high_H);nh_private.getParam("high_S",high_S);nh_private.getParam("high_V",high_V);
        nh_private.getParam("low_H",low_H);nh_private.getParam("low_S",low_S);nh_private.getParam("low_V",low_V);
        nh_private.getParam("scale_factor",scale_factor_);
    }   
} // roboiitk::CentreDetect
