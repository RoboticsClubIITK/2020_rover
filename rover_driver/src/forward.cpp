#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "std_msgs/Float64.h"

#include <stdlib.h> 
double linear_x;

void see(const geometry_msgs::Twist& msg)
{
     //ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
     linear_x = msg.linear.x;
     std::cout<<msg.linear.x;
} 
int main(int argc, char **argv) {
      //Initializes ROS, and sets up a node
      ros::init(argc, argv, "forward_commands");
      ros::NodeHandle nh;
 
      //std::cout<<"Entre you linear velocity: ";
      //std::cin>>linear_x;


      ros::Subscriber sub = nh.subscribe("rover_cmd", 100, see);
      //ros::spinOnce();
      ros::Publisher pub1=nh.advertise<std_msgs::Float64>("rover/corner_lf_wheel_lf_controller/command", 100);
      ros::Publisher pub2=nh.advertise<std_msgs::Float64>("rover/bogie_left_wheel_lm_controller/command", 100);
      ros::Publisher pub3=nh.advertise<std_msgs::Float64>("rover/corner_lb_wheel_lb_controller/command", 100);
      ros::Publisher pub4=nh.advertise<std_msgs::Float64>("rover/corner_rb_wheel_rb_controller/command", 100);
      ros::Publisher pub5=nh.advertise<std_msgs::Float64>("rover/bogie_right_wheel_rm_controller/command", 100);
      ros::Publisher pub6=nh.advertise<std_msgs::Float64>("rover/corner_rf_wheel_rf_controller/command", 100);
      
      srand(time(0));

      ros::Rate rate(100);

       while(ros::ok()) {
          

           ros::spinOnce();
           std_msgs::Float64 msg1;
           std_msgs::Float64 msg2;

           msg1.data = linear_x;
           msg2.data = -linear_x;

           pub1.publish(msg1);
           pub2.publish(msg1);
           pub3.publish(msg1);
           pub4.publish(msg2);
           pub5.publish(msg2);
           pub6.publish(msg2);

           rate.sleep();
         }
}

