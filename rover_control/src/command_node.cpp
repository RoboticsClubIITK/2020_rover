#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>

float lin_vel;
float ang_vel;

void subs_callback(geometry_msgs::Twist msg)
{
	lin_vel = msg.linear.x;
	ang_vel = msg.angular.z;
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "command");
	ros::NodeHandle nh;
	
	ros::Subscriber sub=nh.subscribe("rover/cmd_vel",1,subs_callback);

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

	ros::Rate loopRate(50);

	while(ros::ok())
	{
		ros::spinOnce();

		std_msgs::Float64 pub_msg;
		pub_msg.data = -lin_vel;

	        centre_right_pub.publish(pub_msg);
		front_right_pub.publish(pub_msg);
		back_right_pub.publish(pub_msg);

		pub_msg.data = lin_vel;

		centre_left_pub.publish(pub_msg);
		front_left_pub.publish(pub_msg);
		back_left_pub.publish(pub_msg);
		
		std_msgs::Float64 turn_msg;
		turn_msg.data = ang_vel;

		//turn_lb_pub.publish(turn_msg);
		turn_lf_pub.publish(turn_msg);

		//turn_msg.data = -ang_vel;

		//turn_rb_pub.publish(turn_msg);		
		turn_rf_pub.publish(turn_msg);

		loopRate.sleep();
	}
	return 0;
}    
