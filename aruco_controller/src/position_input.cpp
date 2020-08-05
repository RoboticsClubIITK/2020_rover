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


// void input_callback(geometry_msgs::Pose position){
//     target_pos.position.x= position.position.x;
//     target_pos.position.y= position.position.y;
//     target_pos.orientation.x= position.orientation.x;
//     target_pos.orientation.y= position.orientation.y;
  
// }
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

//ros::Subscriber sub = nh.subscribe("rover/cmd_pose",10, input_callback);
ros::Subscriber sub1 = nh.subscribe("rover/odom",100, subCallback);                           //subscribe the current position from "rover/odom" topic
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("input_topic",10);
ros::Publisher pub2 = nh.advertise<std_msgs::Int32>("hasreached",10);
ros::Rate loopRate(rate);

geometry_msgs::Twist velocity;
float theta;
std_msgs::Int32 flag;
flag.data=0;
target_pos.position.x=-20;
target_pos.position.y=15;
while(ros::ok()){
    ros::spinOnce();
    

    double dist = sqrt((current_pos.position.x- target_pos.position.x)*(current_pos.position.x- target_pos.position.x) + (current_pos.position.y- target_pos.position.y)*(current_pos.position.y- target_pos.position.y));
    double yaw=getyaw();
    theta=atan((target_pos.position.y-current_pos.position.y)/(target_pos.position.x-current_pos.position.x));
    float turnAngle=yaw-theta;
    //if(turnAngle<0){turnAngle=-turnAngle;}
    cout<<"turnangle="<<turnAngle<<"\t"<<"dist="<<dist<<"\n";
    if(dist<0.1){velocity.linear.x=0;velocity.angular.z= 0;}
    else{
        if(abs(turnAngle)<M_PI/45){
            velocity.linear.x=10*dist;
            if(abs(turnAngle)<M_PI/60) velocity.angular.z= 0;
        }
        else{
            velocity.linear.x=0;
            velocity.angular.z= 500*turnAngle;
        }
    }
    if(dist<0.5) flag.data=1;
    pub.publish(velocity);
    pub2.publish(flag);
    loopRate.sleep();
}
return 0;
}
