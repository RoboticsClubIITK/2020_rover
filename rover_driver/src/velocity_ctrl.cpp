#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "std_msgs/Float64.h"

#include <stdlib.h> 

geometry_msgs::Twist vel;


void callBack(const geometry_msgs::Twist& msg){
      vel.linear.x = msg.linear.x;
      vel.angular.z = msg.angular.z;
      std::cout<<msg.linear.x;
      std::cout<<msg.angular.z;
}

int main(int argc, char **argv) {
      ros::init(argc, argv, "velocity_commands");
      ros::NodeHandle nh;

      ros::Subscriber sub = nh.subscribe("cmd_vel", 100, callBack);
      ros::Publisher pub =nh.advertise<geometry_msgs::Twist>("rover_cmd", 100);

      srand(time(0));
      ros::Rate rate(10);

        while(ros::ok()) {
           ros::spinOnce();
           geometry_msgs::Twist msg;
           msg.linear.x = vel.linear.x;
           msg.angular.z= vel.angular.z;
           pub.publish(msg);
           
           rate.sleep();
         }
}