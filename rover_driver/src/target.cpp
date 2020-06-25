#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>


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
int main(int argc, char **argv) {
    ros::init(argc, argv, "target_rover");
    ros::NodeHandle nh;

    ros::Subscriber sub =nh.subscribe("/rover/odom", 100, callBack);
    ros::Subscriber sub1 = nh.subscribe("/aruco_centre", 100, callBackAruco);
    ros::Publisher pub =nh.advertise<geometry_msgs::Point>("/rover/target", 100);

    ros::Rate rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Point goal;

        if((pose2d.x - (-20.0951))<0.1 && (pose2d.y - 15.348)<0.1 && count!=0){
            goal.x = pose2d_aruco_centre.x;
            goal.y = pose2d_aruco_centre.y;
        }
        else {
            goal.x = -20.0951;    //coordinates for flag
            goal.y = 15.348;
            count++;
        } 
        pub.publish(goal);
        rate.sleep();
    }
    return 0;    
}