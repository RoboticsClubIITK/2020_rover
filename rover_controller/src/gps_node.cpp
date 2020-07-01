#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace GeographicLib;

double curr_lat,curr_lon,dest_lat,dest_lon,curr_x,curr_y;

void bot_gps (sensor_msgs::NavSatFix msg)
{
	curr_lat = msg.latitude;
	curr_lon = msg.longitude;
}

void dest_gps (sensor_msgs::NavSatFix msg)
{
        dest_lat = msg.latitude;
	dest_lon = msg.longitude;
}

void curr_Pose_cb (nav_msgs::Odometry msg)
{
	curr_x = msg.pose.pose.position.x;
        curr_y = msg.pose.pose.position.y;	
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "gps");
	ros::NodeHandle nh;

	ros::Subscriber bot_gps_sub = nh.subscribe("/fix", 10, bot_gps);
	ros::Subscriber dest_gps_sub = nh.subscribe("/dest_gps",10, dest_gps);
	ros::Subscriber bot_Pose_sub = nh.subscribe("/rover/odom", 10, curr_Pose_cb);

	ros::Publisher dest_pub = nh.advertise<geometry_msgs::Pose>("/rover/cmd_pose", 10);
	ros::Rate rate(10);

	while(ros::ok())
	{
		ros::spinOnce;
		
		double x1,y1,x2,y2,x,y;
		bool northp;
		int zone;

		UTMUPS::Forward(dest_lat,dest_lon,zone,northp,x2,y2);			      UTMUPS::Forward(curr_lat,curr_lon,zone,northp,x1,y1);		
		x = curr_x + x2-x1;
		y = curr_y + y2-y1;

		geometry_msgs::Pose dest;

		dest.position.x = x;
		dest.position.y = y;

		dest_pub.publish(dest);

		rate.sleep();
	}	
	return 0;
}	
