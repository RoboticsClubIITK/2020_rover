#include <ros/ros.h>
#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <opencv2/aruco.hpp>



void callBack(const nav_msgs::Odometry::ConstPtr msg){

}


int main(int argc, char **argv) {
      ros::init(argc, argv, "aruco_detector");
      ros::NodeHandle nh;

      ros::Subscriber sub = nh.subscribe("/rover/camera/camera_info", 100, callBack);
      //ros::Publisher pub =nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
      
      ros::Rate rate(10);

        while(ros::ok()) {
            ros::spinOnce();


            rate.sleep();
        }
}