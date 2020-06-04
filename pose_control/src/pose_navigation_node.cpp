#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include<stdlib.h>

#define _USE_MATH_DEFINES
#include<cmath>

double dest_x,dest_y,dest_z;
double curr_x,curr_y , curr_z , turn_z;
void cmd_subs_callback (geometry_msgs::Pose msg)
{
	dest_x = msg.position.x;
	dest_y = msg.position.y;
	dest_z = msg.position.z;
}	
void fb_subs_callback(nav_msgs::Odometry msg)
{
	curr_x = msg.pose.pose.position.x;
	curr_y = msg.pose.pose.position.y;
	curr_z = msg.pose.pose.position.z;

	turn_z = msg.pose.pose.orientation.z;
}	
int main(int argc , char** argv)
{
	ros::init(argc , argv , "pose_navigation");
	ros::NodeHandle nh;

	ros::Subscriber cmd_sub = nh.subscribe("rover/cmd_pose",1,cmd_subs_callback);
	ros::Subscriber fb_sub = nh.subscribe("rover/odom",1,fb_subs_callback);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("rover/cmd_vel",1);

	ros::Rate loopRate(30);
	
	double turn_req;

	while(ros::ok())
	{
		//ros::spinOnce();

		geometry_msgs::Twist turn_msg;
		do  //turn
		{
			 ros::spinOnce();

			//fb_subs_callback(fb_sub);
			if(dest_x - curr_x<0)
			{
				if(dest_y - curr_y >= 0) turn_req = atan((dest_y - curr_y) /(dest_x - curr_x)) + M_PI;
				else turn_req = atan((dest_y - curr_y) /(dest_x - curr_x)) - M_PI;	 	
			}
			else if (dest_x - curr_x>0)
			{
			        if(dest_y - curr_y >= 0) turn_req = atan((dest_y - curr_y) /(dest_x - curr_x)) ;
				else turn_req = atan((dest_y - curr_y) /(dest_x - curr_x));      
			}
			else if (dest_y-curr_y>=0) turn_req = M_PI_2;
			else turn_req = -M_PI_2;
			turn_msg.linear.x = 2;
			if(turn_req - turn_z > 0 )
			{
				turn_msg.angular.z = 5;
			}
			else if(turn_req - turn_z < 0)
			{
				turn_msg.angular.z = -5;
			}
			pub.publish(turn_msg);
		}
		while(turn_req - turn_z != 0);
		turn_msg.linear.x = 0;
		turn_msg.angular.z = 0;
		pub.publish(turn_msg);


		loopRate.sleep();	
	}	
	return 0;
}	
