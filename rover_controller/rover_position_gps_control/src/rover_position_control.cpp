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
    void rover_pose::poseCb(const nav_msgs::Odometry& odom_msg){
        x_c=odom_msg.pose.pose.position.x;
        y_c=odom_msg.pose.pose.position.y;
    }
    void rover_pose::cmd_poseCb(const geometry_msgs::Pose& pose_msg) {
        x=pose_msg.position.x;
        y=pose_msg.position.y;
        dist=sqrt(pow(x-x_c,2)+pow(y-y_c,2));
        angle = (heading+atan((y-y_c)/(-(x-x_c))))*180/(2*acos(0.0));
        if(x_c-x<0)
        {
        if(heading<0)
        angle+=180;
        else
        angle-=180;
        }
        if(dist<0.2){
            speed=0;
            ang_vel=0;
        }
        else 
        {
            if(abs(angle)<4)
            {
                speed=10*dist;
                if(abs(angle)<2)
                ang_vel=0;  
            }
            else
            {
                speed=0;
                ang_vel=angle; 
            }
            msg.linear.x=speed;
            msg.angular.z=ang_vel;
            pub_vel.publish(msg);
        }
    }
}//namespace roboiitk::pose_control