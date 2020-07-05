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
using namespace std;
geometry_msgs::Pose target_pos;
geometry_msgs::Pose current_pos;
std_msgs::Int32 flag;
void flag_callback(std_msgs::Int32 flag1){
    flag.data=flag1.data;
}


void subCallback(nav_msgs::Odometry position){
    current_pos.position.x= position.pose.pose.position.x;
    current_pos.position.y= position.pose.pose.position.y;
    current_pos.orientation.x= position.pose.pose.orientation.x;
    current_pos.orientation.y= position.pose.pose.orientation.y;
    current_pos.orientation.z= position.pose.pose.orientation.z;
    current_pos.orientation.w= position.pose.pose.orientation.w;

 return;
 }
double getyaw(){
    tf::Quaternion q(current_pos.orientation.x,current_pos.orientation.y,current_pos.orientation.z,current_pos.orientation.w);
    tf::Matrix3x3 rot(q);
    double roll,pitch,yaw;
    rot.getRPY(roll,pitch,yaw);
    //return yaw;
    if(current_pos.position.x<target_pos.position.x){
        if(yaw>0) yaw -= M_PI;
        else yaw += M_PI;
    }
    return yaw;
}
int main(int argc, char** argv){
ros::init(argc, argv, "pose_control");
ros::NodeHandle nh;
double rate;
nh.getParam("rate",rate);
ros::Subscriber sub2 = nh.subscribe("flag_topic",10, flag_callback);
ros::Subscriber sub1 = nh.subscribe("rover/odom",100, subCallback);                           //subscribe the current position from "rover/odom" topic
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("input_topic",10);
ros::Rate loopRate(rate);

geometry_msgs::Twist velocity;
float theta;
target_pos.position.x=-95;
target_pos.position.y=40;
while(ros::ok()){
    ros::spinOnce();
    

    double dist = sqrt((current_pos.position.x- target_pos.position.x)*(current_pos.position.x- target_pos.position.x) + (current_pos.position.y- target_pos.position.y)*(current_pos.position.y- target_pos.position.y));
    double yaw=getyaw();
    theta=atan((target_pos.position.y-current_pos.position.y)/(target_pos.position.x-current_pos.position.x));
    float turnAngle=yaw-theta;
    cout<<"turnangle="<<turnAngle<<"\t"<<"dist="<<dist<<"\n";
    if(dist<1){velocity.linear.x=0;velocity.angular.z= 0;}
    else{
        if(abs(turnAngle)<M_PI/45){
            velocity.linear.x=10*dist;
            if(abs(turnAngle)<M_PI/60) velocity.angular.z= 0;
        }
        else{
            velocity.linear.x=0;
            velocity.angular.z= 100*turnAngle;
        }
    }
    
    if(flag.data==0){pub.publish(velocity);}
    loopRate.sleep();
}
return 0;
}
