#include <rover_aruco_detector/aruco_marker_detect.hpp>

namespace roboiitk::ArucoDetect {

void ArucoDetect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    src_ = cv_ptr->image;
    ROS_ASSERT(src_.empty() != true);
    findCentre(src_);
}

void ArucoDetect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    image_sub_ = nh.subscribe("/rover/camera/image_raw", 1, &ArucoDetect::imageCb, this);
    // aruco_marker_pub_=nh.advertise<sensor_msgs::Image>("aruco_marker_detect", 1);
    aruco_marker_centre_pub_ = nh.advertise<geometry_msgs::Point>("aruco_marker_centre", 1);
}

void ArucoDetect::findCentre(cv::Mat& src_) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(src_, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    src_.copyTo(aruco_marker_drawing_);
    if (markerIds.size() > 0) {
        cv::aruco::drawDetectedMarkers(aruco_marker_drawing_, markerCorners, markerIds);

        distance_ = -12;  // scale factor
        center_.x = (markerCorners.at(0).at(0).x + markerCorners.at(0).at(1).x + markerCorners.at(0).at(2).x + markerCorners.at(0).at(3).x) / 4;
        center_.y = (markerCorners.at(0).at(0).y + markerCorners.at(0).at(1).y + markerCorners.at(0).at(2).y + markerCorners.at(0).at(3).y) / 4;
        center_.z = distance_;
        aruco_marker_centre_pub_.publish(center_);
        ROS_INFO("Aruco Marker Detected");

        aruco_marker_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", aruco_marker_drawing_).toImageMsg());
        cv::imshow("out", aruco_marker_drawing_);
        cv::waitKey();
        markerIds.clear();
        markerCorners.clear();
    }
}

}  // namespace roboiitk::ArucoDetect
