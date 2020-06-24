#pragma once
#define PI 3.14159265

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace roboiitk::gps_world{

    class rover_gps{
        public:
        void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
        void run();
        void gpsCb(const sensor_msgs::NavSatFix& gps_msg);
        void cmd_gpsCb(const sensor_msgs::NavSatFix& gps_msg);
        void poseCb(const nav_msgs::Odometry& odom_msg); 

        private:
        ros::Subscriber cmd_gps_sub,pose_sub;
        ros::Subscriber gps_sub;
        ros::Publisher cmd_pose_pub;
        ros::Publisher odom_pub;
        const  GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        double orig_lat=49.8999599312;double origin_lon=8.90009587386;
        double curr_utm_x,curr_utm_y,des_utm_x,des_utm_y;
        double odom_x,odom_y,cmd_x,cmd_y;
        double x_c,y_c;
        geometry_msgs::Pose cmd_msg,odom_msg;
    };
}//namespace roboiitk::gps_world