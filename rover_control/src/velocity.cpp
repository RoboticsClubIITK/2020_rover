#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/String.h>
#include <stdlib.h>

std_msgs::Float64 velocity_1, velocity_2;

void callBack(const geometry_msgs::Twist& msg) {
    float sep = 0.3;
    float omega = msg.angular.z;
    velocity_1.data = msg.linear.x + omega * sep / 2;
    velocity_2.data = -(msg.linear.x - omega * sep / 2);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_commands");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("cmd_vel", 100, callBack);

    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("rover/corner_lf_wheel_lf_controller/command", 100);    // left front wheel
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("rover/bogie_left_wheel_lm_controller/command", 100);   // left mid wheel
    ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("rover/corner_lb_wheel_lb_controller/command", 100);    // left back wheel
    ros::Publisher pub4 = nh.advertise<std_msgs::Float64>("rover/corner_rb_wheel_rb_controller/command", 100);    // right back wheel
    ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("rover/bogie_right_wheel_rm_controller/command", 100);  // right mid wheel
    ros::Publisher pub6 = nh.advertise<std_msgs::Float64>("rover/corner_rf_wheel_rf_controller/command", 100);    // right front wheel

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        std_msgs::Float64 msg1;
        std_msgs::Float64 msg2;

        msg1.data = velocity_1.data;
        msg2.data = velocity_2.data;

        pub1.publish(msg1);
        pub2.publish(msg1);
        pub3.publish(msg1);
        pub4.publish(msg2);
        pub5.publish(msg2);
        pub6.publish(msg2);

        rate.sleep();
    }
    return 0;
}
