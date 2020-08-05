#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

float cx = 180.5, cy = 120.5, fx = 236.77842459886676, fy = 236.77842459886676;  // camera properties

ros::Publisher pub;
image_transport::Publisher image_pub_;
geometry_msgs::Pose2D msgs;
geometry_msgs::Pose2D coordinates;

void center(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat src = cv_ptr->image;
    Mat img;
    cvtColor(src, img, COLOR_BGR2GRAY);

    threshold(img, img, 250, 255, THRESH_BINARY_INV);
    threshold(img, img, 250, 255, THRESH_BINARY_INV);

    Moments m = moments(img, true);
    sensor_msgs::ImagePtr msgd = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    image_pub_.publish(msgd);

    msgs.x = (m.m10 / m.m00);
    msgs.y = (m.m01 / m.m00);
    coordinates.x = (msgs.x - cx) / fx;
    coordinates.y = (msgs.y - cy) / fy;

    pub.publish(coordinates);
}

// void frame_transformer(const sensor_msgs::CameraInfo& info) {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "box_detect_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;

    image_pub_ = it.advertise("/image_converter/output_video", 1);
    image_sub = it.subscribe("/rover/camera/image_raw", 1, center);  // topic for rover's camera feed

    pub = nh.advertise<geometry_msgs::Pose2D>("coordinates", 1);  // topic for detected and tranformed coordinates

    ros::Rate loopRate(30);

    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
