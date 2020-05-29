#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
std_msgs::Float64 output_linear,output_angular;
void callback(geometry_msgs::Twist velocity){
    geometry_msgs::Twist velocity2;
    velocity2=velocity;
    output_linear.data=velocity2.linear.x;
    output_angular.data=velocity2.angular.z;
}

int main(int argc, char** argv){
ros::init(argc, argv, "vel_control");
ros::NodeHandle nh;

// std::String topic;
double rate;
// nh.getParam("input_topic_name",topic);
nh.getParam("rate",rate);
ros::Subscriber sub = nh.subscribe("input_topic",10, callback);
ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/rover/bogie_right_wheel_rm_controller/command",10);
ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/rover/bogie_left_wheel_lm_controller/command",10);
ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("/rover/corner_lf_wheel_lf_controller/command",10);
ros::Publisher pub4 = nh.advertise<std_msgs::Float64>("/rover/corner_lb_wheel_lb_controller/command",10);
ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("/rover/corner_rb_wheel_rb_controller/command",10);
ros::Publisher pub6 = nh.advertise<std_msgs::Float64>("/rover/corner_rf_wheel_rf_controller/command",10);

ros::Publisher pub7 = nh.advertise<std_msgs::Float64>("/rover/bogie_left_corner_lf_controller/command",10);
ros::Publisher pub8 = nh.advertise<std_msgs::Float64>("/rover/bogie_right_corner_rf_controller/command",10);
ros::Publisher pub9 = nh.advertise<std_msgs::Float64>("/rover/rocker_left_corner_lb_controller/command",10);
ros::Publisher pub10 = nh.advertise<std_msgs::Float64>("/rover/rocker_right_corner_rb_controller/command",10);
ros::Rate loopRate(rate);
output_angular.data = output_angular.data*rate/60;
while(ros::ok()){
ros::spinOnce();
pub1.publish(output_linear);
pub2.publish(output_linear);
pub3.publish(output_linear);
pub4.publish(output_linear);
pub5.publish(output_linear);
pub6.publish(output_linear);
pub7.publish(output_angular);
pub8.publish(output_angular);
pub9.publish(output_angular);
pub10.publish(output_angular);
loopRate.sleep();
}
return 0;}

