#include <rover_vel_control/vel_cntrl.hpp>

namespace roboiitk::vel_control{
    void rover::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
        vel_sub=nh.subscribe("cmd_vel", 1, &rover::velCb, this);
        pub_lfb=nh.advertise<std_msgs::Float64>("left_back", 1);
        pub_lfm=nh.advertise<std_msgs::Float64>("left_middle", 1);
        pub_lff=nh.advertise<std_msgs::Float64>("left_front", 1);
        pub_rgb=nh.advertise<std_msgs::Float64>("right_back", 1);
        pub_rgm=nh.advertise<std_msgs::Float64>("right_middle", 1);
        pub_rgf=nh.advertise<std_msgs::Float64>("right_front", 1);
        pub_jlf=nh.advertise<std_msgs::Float64>("joint_left_front", 1);
        pub_jlb=nh.advertise<std_msgs::Float64>("joint_left_back", 1);
        pub_jrf=nh.advertise<std_msgs::Float64>("joint_right_front", 1);
        pub_jrb=nh.advertise<std_msgs::Float64>("joint_right_back", 1);
    }
    void rover::velCb(const geometry_msgs::Twist& vel_msg) {
        vel=vel_msg.linear.x;
        ang_vel=50*((vel_msg.angular.z)*acos(0.0)/90);
        msg_lfb.data=vel+ang_vel*sep/2;msg_lfm.data=vel+ang_vel*sep/2;msg_lff.data=vel+ang_vel*sep/2;
        msg_rgb.data=-(vel-ang_vel*sep/2);msg_rgm.data=-(vel-ang_vel*sep/2);msg_rgf.data=-(vel-ang_vel*sep/2);
    }
    void rover::run() {
        pub_lfb.publish(msg_lfb);pub_lfm.publish(msg_lfm);
        pub_lff.publish(msg_lff);pub_rgb.publish(msg_rgb);
        pub_rgm.publish(msg_rgm);pub_rgf.publish(msg_rgf);
    }
} //namespace roboiitk:vel_control
