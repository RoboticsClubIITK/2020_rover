#include <stdlib.h> 
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 pose3d;
void callBack(const geometry_msgs::PoseArray::ConstPtr& msg){
    msg->poses;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_rover");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("/seqOfWayPoints", 100, callBack);
    ros::Publisher pub =nh.advertise<geometry_msgs::Point>("/rover/target", 100);

    ros::Rate rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Point goal;

        pub.publish(goal);
        rate.sleep();
    }
    return 0;    
}