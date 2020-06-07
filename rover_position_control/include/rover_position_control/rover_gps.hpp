#pragma once
#define PI 3.14159265

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/Geodesic.hpp>

namespace roboiitk::gps_world{

    class rover_gps{
        public:
        void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
        void run();
        void gpsCb(const sensor_msgs::NavSatFix& gps_msg);

        private:
        ros::Subscriber gps_sub;
        ros::Publisher cmd_pose_pub;
        double orig_lat=43.642;double origin_lon=-79.380;
        double cmd_x,cmd_y;
        const  GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        geometry_msgs::Pose cmd_msg;
    };
}//namespace roboiitk::gps_world