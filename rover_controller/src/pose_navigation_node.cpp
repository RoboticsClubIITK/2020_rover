#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include<stdlib.h>
#include<cmath>
#include<tf/transform_datatypes.h>

#define _USE_MATH_DEFINES


double dest_x,dest_y,dest_z;
double curr_x,curr_y , curr_z ;
double roll , pitch, yaw, speed, f_angle;

void cmd_subs_callback (geometry_msgs::Pose msg)
{
	dest_x = msg.position.x;
	dest_y = msg.position.y;
	//dest_z = msg.position.z;
}	
void fb_subs_callback(nav_msgs::Odometry msg)
{
	curr_x = msg.pose.pose.position.x;
	curr_y = msg.pose.pose.position.y;
	//curr_z = msg.pose.pose.position.z;

	tf::Quaternion q (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w
	);

	tf::Matrix3x3 m(q);
	//double roll,pitch,yaw;

	m.getRPY(roll,pitch,yaw);

	//angle = -yaw;
}	
int main(int argc , char** argv)
{
	ros::init(argc , argv , "pose_navigation");
	ros::NodeHandle nh;

	ros::Subscriber cmd_sub = nh.subscribe("rover/cmd_pose",10,cmd_subs_callback);
	ros::Subscriber fb_sub = nh.subscribe("rover/odom",10,fb_subs_callback);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("rover/cmd_vel",10);

	ros::Rate loopRate(10);
	
	//double turn_req;

	while(ros::ok())
	{
		ros::spinOnce();
		
		float dist,angle;

		dist = sqrt((dest_x-curr_x)*(dest_x-curr_x) + (dest_y-curr_y)*(dest_y-curr_y) + (dest_z-curr_z)*(dest_z-curr_z));
		
		//float ang1;
		angle = yaw + atan(dest_y-curr_y)/(-(dest_x-curr_x));

		if((curr_x-dest_x)<0)
		{
			if(yaw<0)
				angle += M_PI;
			else
				angle -= M_PI;	
		}
		

		//speed = dist*10;

		//if(abs(angle)<0.05)
		//	angle = 0;
		if(dist<0.3)
		{
			speed = 0;
			angle = 0;
		}	
		else//(abs(angle)<0.07)
		{
			if(abs(angle)<0.07)
			{	
				speed = dist*10;
				if(abs(angle)<0.035)
					f_angle = 0;
			}
			else
			{
				speed = 0;
				f_angle = angle;
			}	

		}	

		geometry_msgs::Twist msg;
		msg.linear.x = speed;
		msg.angular.z = f_angle;
		pub.publish(msg);


		loopRate.sleep();	
	}	
	return 0;
}	
