#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Pose.h>
#include<geometry_msgs/Twist.h>

namespace roboiitk::obstacle_avoid{

    class obstacleAvoid{
           public:
           void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
           void run();
           void poseCb(const nav_msgs::Odometry& odom_msg);
           void headingCb(const geometry_msgs::Vector3& head_msg);
           void laserCb(const sensor_msgs::LaserScan& laser_msg);
           void setGoal();
           void moveForward();
           void moveRight();
           void moveLeft();
           void stop();

           private:
           ros::Publisher pub_vel;
           ros::Subscriber heading_sub,pose_sub,laser_sub;
           geometry_msgs::Twist speed_msg;geometry_msgs::Point goal;
           double heading,tar_heading,x_c,y_c;
           double angle,dist,ang_vel;
           double obstacle_min_distance_threshold,angle_threshold_low,angle_threshold_high,distance_threshold,heading_threshold;
           float min_distance,min_distance_angle=0;
           float range[360];
    };
}//namespace roboiitk::obstacle_avoid
