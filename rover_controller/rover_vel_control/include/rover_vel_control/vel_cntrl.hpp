#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace roboiitk::vel_control{

    class rover{
           public:
           void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
           void run();
           void velCb(const geometry_msgs::Twist& vel_msg);

           private:
           ros::Publisher pub_lfb,pub_lfm,pub_lff,pub_rgb,pub_rgm,pub_rgf,pub_jlf,pub_jlb,pub_jrf,pub_jrb;
           ros::Subscriber vel_sub;
           geometry_msgs::Twist msg;
           double vel,ang_vel;double sep=0.3;
           const float const_turn_angle=0.1;
           std_msgs::Float64 msg_lfb,msg_lfm,msg_lff,msg_rgb,msg_rgm,msg_rgf,msg_jlf,msg_jlb,msg_jrf,msg_jrb;
    };
}//namespace roboiitk::vel_control