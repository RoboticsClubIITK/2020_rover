#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>


int main(int argc, char** argv){
ros::init(argc, argv, "vel_input");
ros::NodeHandle nh;

// std::String topic;
std_msgs::Float64 linear_vel,angular_vel;
double rate;
nh.getParam("rate",rate);
nh.getParam("linear_vel",linear_vel.data);
nh.getParam("angular_vel",angular_vel.data);
// nh.getParam("input_topic_name",topic);
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("input_topic",10);
geometry_msgs::Twist velocity;
velocity.linear.x= linear_vel.data;
velocity.angular.z=angular_vel.data;
ros::Rate loopRate(rate);
while(ros::ok()){
ros::spinOnce();
pub.publish(velocity);
loopRate.sleep();
}
return 0;}
