#pragma once

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

namespace rover::planner {

class publish_waypoint
{
public:
    void odomCb(const nav_msgs::Odometry& odom_msg);
    void pathCb(const geometry_msgs::PoseArray& path_msg);
    void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
    void run();

private:
    ros::Publisher pub_waypoint_;
    ros::Subscriber odom_sub_,path_sub_;
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::PoseArray path_msg_;
    geometry_msgs::Pose cmd_msg_;
    int id=-1;
    double dist_;
};

}  // namespace rover::planner