#include<rover_position_control/rover_pose_control.hpp>

namespace roboiitk::pose_control{
    void rover_pose::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
        cmd_pose_sub=nh.subscribe("rover/cmd_pose", 1, &rover_pose::cmd_poseCb, this);
        heading_sub=nh.subscribe("rover/heading", 1, &rover_pose::headingCb, this);
        pub_vel=nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        pose_sub=nh.subscribe("rover/odom", 1, &rover_pose::poseCb, this);
    }
    void rover_pose::headingCb(const geometry_msgs::Vector3& head_msg){
        heading=head_msg.z;
    }
    void rover_pose::cmd_poseCb(const geometry_msgs::Pose& pose_msg) {
        x=pose_msg.position.x;
        y=pose_msg.position.y;
        if(x<=0)tar_heading=atan(y/x);
        if(y>=0&&x>=0)tar_heading=atan(y/x)-PI;
        if(y<0&&x>=0)tar_heading=-atan(y/x)+PI;

        ROS_INFO("published target_heading:  yaw=%lf",  tar_heading);


        if(fabs(tar_heading-heading)<0.05){
            if(count>=20){flag_start=1;return;}
            msg.linear.x=0.1;
            msg.angular.z=k_ang_vel*(fabs(tar_heading-heading));
            pub_vel.publish(msg);count++;return;}

        /***** Checking whether to rotate cw or acw based on the orientations ***/
        if(tar_heading*heading>=0){
            if(tar_heading>=heading){
                msg.linear.x=vel;
                msg.angular.z=k_ang_vel*(fabs(tar_heading-heading));
                pub_vel.publish(msg);       
            }
            else{
                msg.linear.x=vel;
                msg.angular.z=-1*k_ang_vel*(fabs(tar_heading-heading));
                pub_vel.publish(msg);
            }
        }
        else{
            if(tar_heading>=heading){
                if(fabs(tar_heading)+fabs(heading)<=2*PI-(fabs(tar_heading)+fabs(heading))){
                msg.linear.x=vel;
                msg.angular.z=k_ang_vel*(fabs(tar_heading-heading));
                pub_vel.publish(msg);
                }
                else{
                msg.linear.x=vel;
                msg.angular.z=-1*k_ang_vel*(fabs(tar_heading-heading));
                pub_vel.publish(msg);
                }       
            }
            else{
                if(fabs(tar_heading)+fabs(heading)<=2*PI-(fabs(tar_heading)+fabs(heading))){
                msg.linear.x=vel;
                msg.angular.z=-1*k_ang_vel*(fabs(tar_heading-heading));
                pub_vel.publish(msg);
                }
                else{
                msg.linear.x=vel;
                msg.angular.z=k_ang_vel*(fabs(tar_heading-heading));
                pub_vel.publish(msg);
                }     
            }
        }
        //ROS_INFO("angular vel: %lf linear vel: %lf",  vel,ang_vel);
    }
    void rover_pose::poseCb(const nav_msgs::Odometry& odom_msg){
        x_c=odom_msg.pose.pose.position.x;
        y_c=odom_msg.pose.pose.position.y;
        if(flag_start){
            msg.linear.x=k_vel*(sqrt(pow(x-x_c,2)+pow(y-y_c,2)));
            msg.angular.z=0;
        }
        if(sqrt(pow(x-x_c,2)+pow(y-y_c,2))<0.01){
            msg.linear.x=0;
            msg.angular.z=0;
            flag_start=0;count=0;
        }
        pub_vel.publish(msg);
    }
}//namespace roboiitk::pose_control