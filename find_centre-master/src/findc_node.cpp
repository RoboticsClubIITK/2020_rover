#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <find_centre/Centre.h>

cv::Mat input_image;

void imgCallback(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr image_ptr;
    
    try
    {
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        input_image = image_ptr->image;
    }
    catch(cv_bridge::Exception& error)
    {
        ROS_ERROR ("cv_bridge exception: %s", error.what());
    }
    return;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "findc");
    
    ros::NodeHandle nh;
    
    bool overlay = false;
    bool debug = false;
    double rate = 10;
    int h_min = 40;
    int h_max = 80;
    
    nh.param("overlay", overlay, overlay);
    nh.param("debug", debug, debug);
    nh.param("rate", rate, rate);
    nh.param("h_min", h_min, h_min);
    nh.param("h_max", h_max, h_max);
    
    ros::Rate loop_rate(rate);
    
    ros::Subscriber img_sub = nh.subscribe("/magnus/camera/image_raw", 1, imgCallback);
    
    ros::Publisher centre_pub = nh.advertise<find_centre::Centre>("findc/centre", 1);
    ros::Publisher thresh_pub = nh.advertise<sensor_msgs::Image>("findc/thresh", 1);
    ros::Publisher contour_pub = nh.advertise<sensor_msgs::Image>("findc/contours", 1);
    ros::Publisher processed_pub = nh.advertise<sensor_msgs::Image>("findc/processed", 1);
    
    cv::Scalar hsv_min(h_min,0,0);
    cv::Scalar hsv_max(h_max,255,255);
    
    while(ros::ok())
    {
        ros::spinOnce();
        if(input_image.empty()) continue;
        
        cv::Mat hsv_img;
        cv::cvtColor(input_image, hsv_img, CV_BGR2HSV);
        
        cv::Mat thresh_img;
        cv::inRange(hsv_img,hsv_min,hsv_max,thresh_img);
        if(debug)
        {
            sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",thresh_img).toImageMsg();
            thresh_pub.publish(thresh_msg);
        }
        
        cv::Mat smooth_image;
        cv::morphologyEx(thresh_img, smooth_image, CV_MOP_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(smooth_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        
        int box_contour_index = 0;
        double max_area = 0.0;
        for(int i=0;i<contours.size();i++)
        {
            double area = cv::contourArea(contours[i]);
            if(area > max_area && contours[i].size() == 4)
            {
                box_contour_index = i;
                max_area = area;
            }
        }
        
        if(debug)
        {
            cv::Mat drawing = cv::Mat::zeros(input_image.size(), CV_8UC3);
            if(overlay) {drawing = input_image;}
            cv::drawContours(drawing, contours, box_contour_index,cv::Scalar(0,0,255));
            sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
            contour_pub.publish(contour_msg);
        }
        
        double centre_x = 0.0;
        double centre_y = 0.0;
        std::vector<cv::Point> contour = contours[box_contour_index];
        for (int i=0;i<contour.size();i++)
        {
            centre_x += contour[i].x;
            centre_y += contour[i].y;
        }
        centre_x /= contour.size();
        centre_y /= contour.size();
        
        find_centre::Centre centre_msg;
        centre_msg.header.stamp = ros::Time::now();
        centre_msg.x = centre_x;
        centre_msg.y = centre_y;
        centre_pub.publish(centre_msg);
        
        if(debug)
        {
            cv::Mat final_img = cv::Mat::zeros(input_image.size(), CV_8UC3);
            if(overlay)
            {
                final_img = input_image;
                cv::drawContours(final_img,contours,box_contour_index,cv::Scalar(0,0,255));
            }
            cv::circle(final_img,cv::Point(centre_x, centre_y), 3, cv::Scalar(255,0,0));
            sensor_msgs::ImagePtr proc_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();
            processed_pub.publish(proc_msg);
        }
        
        loop_rate.sleep();
    }
    
    return 0;
}  



