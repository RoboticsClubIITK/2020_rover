#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>

geometry_msgs::Pose target_pos;
geometry_msgs::Pose current_pos;


void input_callback(geometry_msgs::Pose position){
    target_pos.position.x= position.position.x;
    target_pos.position.y= position.position.y;
    target_pos.orientation.x= position.orientation.x;
    target_pos.orientation.y= position.orientation.y;
  
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
    return yaw;
}
int main(int argc, char** argv){
ros::init(argc, argv, "pose_control");
ros::NodeHandle nh;
double rate;
nh.getParam("rate",rate);

ros::Subscriber sub = nh.subscribe("rover/cmd_pose",10, input_callback);
ros::Subscriber sub1 = nh.subscribe("rover/odom",100, subCallback);                           //subscribe the current position from "rover/odom" topic
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("input_topic",10);
ros::Rate loopRate(rate);

geometry_msgs::Twist velocity;
float theta;

while(ros::ok()){
    ros::spinOnce();
    double dist = (current_pos.position.x- target_pos.position.x)*(current_pos.position.x- target_pos.position.x) + (current_pos.position.y- target_pos.position.y)*(current_pos.position.y- target_pos.position.y);
    double yaw=getyaw();
    theta=atan((target_pos.orientation.y-current_pos.orientation.y)/(target_pos.orientation.x-current_pos.orientation.x));
    float turnAngle=yaw-theta;
    if(turnAngle>0.1) {velocity.angular.z= 20; velocity.linear.x=0;}
    else {velocity.angular.z=0; velocity.linear.x=100; }
    if(dist<0.01) velocity.linear.x = 0;
    pub.publish(velocity);
    loopRate.sleep();
}
return 0;
}
