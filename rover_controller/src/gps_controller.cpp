#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include <cmath>
#include <exception>
#include<sensor_msgs/NavSatFix.h>

using namespace std;
using namespace GeographicLib;


geometry_msgs::Pose curr_odom;
sensor_msgs::NavSatFix curr_gps;

void callback(sensor_msgs::NavSatFix position){
    curr_gps.latitude=position.latitude;
    curr_gps.longitude=position.longitude;
  }

void subCallback(nav_msgs::Odometry position){
    curr_odom.position.x= position.pose.pose.position.x;
    curr_odom.position.y= position.pose.pose.position.y;
    curr_odom.orientation.x= position.pose.pose.orientation.x;
    curr_odom.orientation.y= position.pose.pose.orientation.y;
    curr_odom.orientation.z= position.pose.pose.orientation.z;
    curr_odom.orientation.w= position.pose.pose.orientation.w;
  }

int main(int argc,char** argv){   
    ros::init(argc, argv, "gps_control");
    ros::NodeHandle nh;
    double rate;
    nh.getParam("rate",rate);  
    ros::Subscriber sub = nh.subscribe("/fix",10, callback);                                     //suscribe curr_position(and convert into curr_utm)
    ros::Subscriber sub1 = nh.subscribe("rover/odom",10, subCallback);                           //subscribe curr_odom
    ros::Publisher pub= nh.advertise<geometry_msgs::Pose>("rover/cmd_pose",10);                  //publishes des_odom         
    ros::Rate loopRate(rate);
    sensor_msgs::NavSatFix des_gps;
    geometry_msgs::Pose des_odom;
    geometry_msgs::Pose des_utm;
    geometry_msgs::Pose curr_utm;

    des_gps.latitude= 50;
    des_gps.longitude= 50;
    while(ros::ok()){
        ros::spinOnce();
        try {
    {     
      int zone;
      bool northp;    
      double c1,k1,c2,k2;
      UTMUPS::Forward(des_gps.latitude, des_gps.longitude, zone,northp, des_utm.position.x, des_utm.position.y,c1,k1);
      UTMUPS::Forward(curr_gps.latitude, curr_gps.longitude, zone,northp, curr_utm.position.x, curr_utm.position.y,c2,k2);
    }
   
  } catch (const exception &e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
        des_odom.position.x=curr_odom.position.x + des_utm.position.x - curr_utm.position.x;
        des_odom.position.y=curr_odom.position.y + des_utm.position.y - curr_utm.position.y;

        pub.publish(des_odom);
        loopRate.sleep();
    }

    return 0;
}
