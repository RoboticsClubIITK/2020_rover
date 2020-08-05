#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

geometry_msgs::Pose2D pose2d;  // pose of rover

void callBack(const nav_msgs::Odometry::ConstPtr msg) {
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose2d.theta = yaw;  // angle of rover w.r.t axis
}

geometry_msgs::Point goal;  // defining goal point
void callBackTarget(const geometry_msgs::Point::ConstPtr msg) {
    goal.x = msg->x;  // initiallising goal point
    goal.y = msg->y;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/rover/odom", 100, callBack);           // getting position of rover
    ros::Subscriber sub1 = nh.subscribe("/rover/target", 100, callBackTarget);  // getting position of target
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Twist speed;

        double inc_x = goal.x - pose2d.x;
        double inc_y = goal.y - pose2d.y;

        double distance = sqrt(inc_x * inc_x + inc_y * inc_y);
        double theta_one = std::atan(inc_y / inc_x);
        double final_theta = pose2d.theta - theta_one;

        if (distance < 0.01) {
            speed.linear.x = 0;  // rover will stop when reaches goal
        }
        /*---------------------*/  // towards flag
        else {
            if (final_theta > 0.08) {
                speed.linear.x = 0.0;
                speed.angular.z = 20 * final_theta;
                std::cout << "Final theta: " << final_theta << std::endl;
                std::cout << "Angular Speed: " << speed.angular.z << std::endl;
            } else if (final_theta < -0.08) {
                speed.linear.x = 0.0;
                speed.angular.z = 20 * final_theta;
                std::cout << "Final theta: " << final_theta << std::endl;
                std::cout << "Angular Speed: " << speed.angular.z << std::endl;
            } else {
                speed.linear.x = 20 * distance;
                speed.angular.z = 0.0;
                std::cout << "Final theta: " << final_theta << std::endl;
                std::cout << "Linear Speed: " << speed.linear.x << std::endl;
            }
            /*---------------------*/
        }
        pub.publish(speed);
        rate.sleep();
    }
    return 0;
}
