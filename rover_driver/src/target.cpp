#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "target_rover");
    ros::NodeHandle nh;

    ros::Publisher pub =nh.advertise<geometry_msgs::Point>("/rover/target", 100);

    ros::Rate rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Point goal;
        goal.x = -20.0951;    //coordinates for flag
        goal.y = 15.348;
        pub.publish(goal);
        rate.sleep();
    }
    return 0;    
}