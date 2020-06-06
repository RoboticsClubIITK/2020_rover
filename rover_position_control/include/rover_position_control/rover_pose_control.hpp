#pragma once
#define PI 3.14159265
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include<geometry_msgs/Twist.h>

namespace roboiitk::pose_control{

    class rover_pose{
           public:
           void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
           void run();
           void cmd_poseCb(const geometry_msgs::Pose& pose_msg);
           void poseCb(const nav_msgs::Odometry& odom_msg);
           void headingCb(const geometry_msgs::Vector3& head_msg);

           private:
           ros::Publisher pub_vel;
           ros::Subscriber cmd_pose_sub,heading_sub,pose_sub;
           geometry_msgs::Twist msg;
           double heading,tar_heading,x,y,x_c,y_c,k_vel=2;
           double vel=0.2;double ang_vel=15;double k_ang_vel=30;
           long flag_start=0;long flag_end=0;int count=0;
    };
}//namespace roboiitk::pose_control
