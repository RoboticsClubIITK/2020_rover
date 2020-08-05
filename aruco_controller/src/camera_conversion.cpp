
#include<std_msgs/Float32.h>
#include<ros/ros.h>
#include<iostream>
#include<geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>


using namespace std;

geometry_msgs::Pose converted;
float cx=180.5 ,cy=120.5 ,fx=236.77842459886676 ,fy=236.77842459886676 ;   //camera parameters
void callback(aruco_controller::Centre point){
    float u=point.x;
    float v=point.y;
    converted.position.x=(u-cx)/fx;
     converted.position.x=(v-cy)/fy;
    cout<<"("<<u<<",\t"<u<<")\n";
}



int main(int argc,char** argv){
    ros::init(argc, argv, "convertor");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("coordinate_topic",10,callback1);
   
    ros::Publisher pub=nh.advertise<geometry_msgs::Pose>("topic2",10);
    double rate;
    nh.getParam("rate",rate);  
    ros::Rate loopRate(rate);
    while(ros::ok()){
        pub.publish(converted);
        loopRate.sleep();
    }
    return 0;
}