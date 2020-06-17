#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>


//double x = 0.0;
//double y = 0.0 ;
//double theta = 0.0;
geometry_msgs::Pose2D pose2d;

void callBack(const nav_msgs::Odometry::ConstPtr msg){
    //x = msg.pose.pose.position.x;
    //y = msg.pose.pose.position.y;
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw;
    std::cout<<yaw;
}

int main(int argc, char **argv) {
      ros::init(argc, argv, "speed_controller");
      ros::NodeHandle nh;

      ros::Subscriber sub = nh.subscribe("/rover/odom", 100, callBack);
      ros::Publisher pub =nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

      geometry_msgs::Point goal;
      goal.x = -20.0951;
      goal.y = 15.348;
      
      srand(time(0));
      ros::Rate rate(10);

        while(ros::ok()) {
            ros::spinOnce();
            geometry_msgs::Twist speed;
            double inc_x = goal.x -pose2d.x;
            double inc_y = goal.y -pose2d.y;
            double theta_one = std::atan2(inc_y, inc_x);
            double final_theta = abs(theta_one - pose2d.theta);
            if (final_theta > 1.2){
                speed.linear.x = 0.0;
                speed.angular.z = 5;
            }
            else{
                speed.linear.x = 1000.0;
                speed.angular.z = 0.0;
            }
            pub.publish(speed);
            rate.sleep();
        }
}