#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>

std_msgs::Float64 a;
float omega,sep,speed,angle;

void subs_callback(geometry_msgs::Twist msg)
{
	speed = msg.linear.x;
	angle = msg.angular.z;
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "command");
	ros::NodeHandle nh;
	
	ros::Subscriber sub=nh.subscribe("rover/cmd_vel",10,subs_callback);

	ros::Publisher centre_right_pub = nh.advertise<std_msgs::Float64>("rover/bogie_right_wheel_rm_controller/command",1);
        ros::Publisher centre_left_pub = nh.advertise<std_msgs::Float64>("rover/bogie_left_wheel_lm_controller/command",1);
        ros::Publisher back_right_pub = nh.advertise<std_msgs::Float64>("rover/corner_rb_wheel_rb_controller/command",1);
	ros::Publisher front_right_pub = nh.advertise<std_msgs::Float64>("rover/corner_rf_wheel_rf_controller/command",1);
	ros::Publisher back_left_pub = nh.advertise<std_msgs::Float64>("rover/corner_lb_wheel_lb_controller/command",1);
	ros::Publisher front_left_pub = nh.advertise<std_msgs::Float64>("rover/corner_lf_wheel_lf_controller/command",1);

	 ros::Publisher turn_lf_pub = nh.advertise<std_msgs::Float64>("rover/bogie_left_corner_lf_controller/command",1);
	 ros::Publisher turn_rf_pub = nh.advertise<std_msgs::Float64>("rover/bogie_right_corner_rf_controller/command",1); 
	 ros::Publisher turn_lb_pub = nh.advertise<std_msgs::Float64>("rover/rocker_left_corner_lb_controller/command",1);
	ros::Publisher turn_rb_pub = nh.advertise<std_msgs::Float64>("rover/rocker_right_corner_rb_controller/command",1);      	 

	ros::Rate loopRate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		sep = 0.3;
		omega=50*angle;

		//std_msgs::Float64 pub_msg;
		a.data = -(speed - omega*sep/2);

	        centre_right_pub.publish(a);
		front_right_pub.publish(a);
		back_right_pub.publish(a);

		a.data = speed + omega*sep/2;

		centre_left_pub.publish(a);
		front_left_pub.publish(a);
		back_left_pub.publish(a);
		
		//std_msgs::Float64 turn_msg;
		//turn_msg.data = ang_vel;

		//turn_lb_pub.publish(turn_msg);
		//rn_lf_pub.publish(turn_msg);

		//turn_msg.data = -ang_vel;

		//turn_rb_pub.publish(turn_msg);		
		//turn_rf_pub.publish(turn_msg);

		loopRate.sleep();
	}
	return 0;
}    
