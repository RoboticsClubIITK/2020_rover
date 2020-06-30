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
    //geometry_msgs::Pose2D pose2d;
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
    //std::cout<<yaw;
}
geometry_msgs::Point goal;
void callBackTarget(const geometry_msgs::Point::ConstPtr msg){
    goal.x = msg->x;
    goal.y = msg->y;
}

int main(int argc, char **argv) {
      ros::init(argc, argv, "speed_controller");
      ros::NodeHandle nh;

      ros::Subscriber sub = nh.subscribe("/rover/odom", 100, callBack);
      ros::Subscriber sub1 = nh.subscribe("/rover/target", 100, callBackTarget);
      ros::Publisher pub =nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

      //geometry_msgs::Point goal;
      //goal.x = -20.0951;
      //goal.y = 15.348;
      
      srand(time(0));
      ros::Rate rate(100);

        while(ros::ok()) {
            ros::spinOnce();
            geometry_msgs::Twist speed;
            
            double inc_x = goal.x -pose2d.x;
            double inc_y = goal.y -pose2d.y;

            double distance = sqrt(inc_x*inc_x + inc_y*inc_y);
            double theta_one = std::atan(inc_y/inc_x);
            double final_theta =  pose2d.theta - theta_one ;
            if (final_theta > 0.1){
                speed.linear.x = 0.0;
                speed.angular.z = 1000;
            }
            else{
                speed.linear.x = 10;
                speed.angular.z = 0.0;
            }

            if(distance <0.01){
                speed.linear.x = 0;
            }
            pub.publish(speed);
            rate.sleep();
        }
}