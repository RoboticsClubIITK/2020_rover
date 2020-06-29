#include <ros/ros.h>
#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aruco_ros/Centre.h>
#include <nav_msgs/Odometry.h>

aruco_ros::Centre centre_msg;
void callBack(const geometry_msgs::Vector3Stamped& msg){
    centre_msg.x = msg.vector.x;
    centre_msg.y = msg.vector.y;
    centre_msg.z = msg.vector.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "try");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("position", 100, callBack);
    ros::Publisher pub =nh.advertise<nav_msgs::Odometry>("/aruco_centre_try", 100);

    ros::Rate rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        pub.publish(centre_msg);
        rate.sleep();
    }
}