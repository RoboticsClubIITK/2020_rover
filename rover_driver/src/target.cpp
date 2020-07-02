#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

int count = 0;
geometry_msgs::Pose2D pose2d;
void callBack(const nav_msgs::Odometry::ConstPtr msg){
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;
}
geometry_msgs::Pose2D pose2d_aruco_centre;
void callBackAruco(const nav_msgs::Odometry::ConstPtr msgAruco){
    pose2d_aruco_centre.x = msgAruco->pose.pose.position.x;
    pose2d_aruco_centre.y = msgAruco->pose.pose.position.y; 
}
geometry_msgs::Vector3 pose3dtry;
void callBackArucoTry(const nav_msgs::Odometry::ConstPtr& msgArucoTry){
    pose3dtry.x = msgArucoTry->pose.pose.position.x;
    pose3dtry.y = msgArucoTry->pose.pose.position.y;
    pose3dtry.z = msgArucoTry->pose.pose.position.z;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "target_rover");
    ros::NodeHandle nh;

    //ros::Subscriber sub =nh.subscribe("/rover/odom", 100, callBack);
    ros::Subscriber sub1 = nh.subscribe("/aruco_centre", 100, callBackAruco);
    ros::Subscriber sub2 = nh.subscribe("/aruco_centre_try", 100, callBackArucoTry);
    ros::Publisher pub =nh.advertise<geometry_msgs::Point>("/rover/target", 100);

    ros::Rate rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Point goal;

        if((pose2d.x - (-97.7568))<0.5 && (pose2d.y - 41.7204)<0.5 && count!=0 && pose3dtry.z!=0){              //aruco assignment
            goal.x = pose3dtry.x; 
            goal.y = pose3dtry.y;  
        }
        else if((pose2d.x - (-97.7568))<0.5 && (pose2d.y - 41.7204)<0.5 && count!=0){
            goal.x = pose2d_aruco_centre.x;
            goal.y = pose2d_aruco_centre.y;
        }
        else {
            goal.x = -97.7568;    //coordinates for flag
            goal.y = 41.7204;
            count++;
        } 
        pub.publish(goal);
        rate.sleep();
    }
    return 0;    
}