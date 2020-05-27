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
        ang_vel=vel_msg.angular.z;
        msg_lfb.data=vel;msg_lfm.data=vel;msg_lff.data=vel;
        msg_rgb.data=-1*vel;msg_rgm.data=-1*vel;msg_rgf.data=-1*vel;

        /*if(vel<0){
            msg_lfb.data=-1*vel;msg_lfm.data=-1*vel;msg_lff.data=-1*vel;
            msg_rgb.data=vel;msg_rgm.data=vel;msg_rgf.data=vel;
        }*/

        msg_jlf.data=-1*ang_vel*const_turn_angle;msg_jlb.data=ang_vel*const_turn_angle;
        msg_jrf.data=-1*ang_vel*const_turn_angle;msg_jrb.data=ang_vel*const_turn_angle;
        
        /*if(ang_vel<0){
            msg_jlf.data=ang_vel*const_turn_angle;msg_jlb.data=-1*ang_vel*const_turn_angle;
            msg_jrf.data=ang_vel*const_turn_angle;msg_jrb.data=-1*ang_vel*const_turn_angle;
        }*/
    }
    void rover::run() {
         pub_lfb.publish(msg_lfb);pub_lfm.publish(msg_lfm);
         pub_lff.publish(msg_lff);pub_rgb.publish(msg_rgb);
         pub_rgm.publish(msg_rgm);pub_rgf.publish(msg_rgf);
         pub_jlf.publish(msg_jlf);pub_jlb.publish(msg_jlb);
         pub_jrf.publish(msg_jrf);pub_jrb.publish(msg_jrb);
    }
} //namespace roboiitk:vel_control
