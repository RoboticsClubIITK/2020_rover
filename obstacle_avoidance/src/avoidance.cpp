#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>
#include<iostream>
#include<math.h>
#include<std_msgs/Int32.h>
#include<sensor_msgs/LaserScan.h>
using namespace std;
geometry_msgs::Twist velocity;
std_msgs::Int32 flag;
void move_fwd(){
    velocity.linear.x=500;
    velocity.angular.z=0;
    
}
void move_right(){
     velocity.angular.z= 800;
     velocity.linear.x=10;
     
}
void lidar_callback(sensor_msgs::LaserScan laser_data){
    float range_max= laser_data.range_max;
    flag.data=0;
    flag1.data=0;

    float min=range_max;
    for(int i=0;i<360;i++){
        if((laser_data.ranges[i]<range_max)&&(laser_data.ranges[i]>0.40)&&(laser_data.ranges[i]<4.0) ) 
        {   flag.data=1;
            int index=i;
            if(laser_data.ranges[i]<min) min=laser_data.ranges[i];
        }
    }
       
}
int main(int argc, char** argv){
ros::init(argc, argv, "lidar_control");
ros::NodeHandle nh;
double rate;
nh.getParam("rate",rate);
ros::Subscriber sub = nh.subscribe("rover/scan",10, lidar_callback);
ros::Publisher pub_flag = nh.advertise<std_msgs::Int32>("flag_topic",10);
ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("input_topic",10);
ros::Rate loopRate(rate);
while(ros::ok()){
    ros::spinOnce();
    cout<<"flag="<<flag.data<<"\n";
    pub_flag.publish(flag);
    if(flag.data==1) {
        move_right();
        pub_vel.publish(velocity);
        loopRate.sleep();
        move_fwd();
        pub_vel.publish(velocity);
        loopRate.sleep();
    }
    loopRate.sleep();
}
return 0;
}
