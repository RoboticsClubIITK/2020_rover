#include <ros/ros.h>
#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <rover_aruco/Centre.h>

rover_aruco::Centre centre_msg;
void callBack(const rover_aruco::Centre& msg){
    centre_msg.x = msg.x;
    centre_msg.y = msg.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_centre");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/detector/centre", 100, callBack);
    ros::Publisher pub =nh.advertise<nav_msgs::Odometry>("/aruco_centre", 100);

    ros::Rate rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        pub.publish(centre_msg);
        rate.sleep();
    }
}