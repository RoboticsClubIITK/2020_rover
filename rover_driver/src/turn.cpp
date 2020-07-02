#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "std_msgs/Float64.h"

#include <stdlib.h> 
double angular_z;

void see(const geometry_msgs::Twist& msg)
{
     angular_z = msg.angular.z;
     std::cout<<msg.angular.z;
} 

int main(int argc, char **argv) {
      ros::init(argc, argv, "turn_commands");
      ros::NodeHandle nh;

      ros::Subscriber sub = nh.subscribe("cmd_vel", 100, see);

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

           msg1.data = angular_z*10;
           msg2.data = angular_z*10;
           

           pub1.publish(msg1);
           pub2.publish(msg1);
           pub3.publish(msg1);
           pub4.publish(msg2);
           pub5.publish(msg2);
           pub6.publish(msg2);

           rate.sleep();
        }
}