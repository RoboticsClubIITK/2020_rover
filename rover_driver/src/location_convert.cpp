#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <stdlib.h> 
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <iostream>
using namespace GeographicLib;
using namespace std;

int zone;
bool northp;
double x, y, g, k;
void saw(const sensor_msgs::NavSatFixConstPtr& msg){
    //ROS_INFO_STREAM("Longitude value: "<< msg->longitude<<" Latitude value: "<<msg->latitude);
    UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, x, y, g, k);         
    string zonestr = UTMUPS::EncodeZone(zone, northp); 
    ROS_INFO_STREAM("X: "<<x<< "Y: "<<y);
    //cout<<x<<"   "<<y;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_value");
    ros::NodeHandle nh;
 
    ros::Subscriber sub = nh.subscribe("fix", 100, saw);

    ros::Rate rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}