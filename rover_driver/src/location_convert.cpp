#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <stdlib.h> 
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <iostream>
#include "navsat_conversions.h"
#include "UTMUPS.hpp"
//#include <GeographicLib/GeoCoords.hpp>
using namespace GeographicLib;
using namespace std;

//geometry_msgs::Point global_position;
int zone;
bool northp;
double x, y, g, k;
void saw(const sensor_msgs::NavSatFixConstPtr& msg){
    ROS_INFO_STREAM("Longitude value: "<< msg->longitude<<" Latitude value: "<<msg->latitude);
    UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, x, y, g, k);           /*Yahi se dekhna abhi baar bas 2 line edit karne, ye aur niche wali, aur ek baar cmakelists bhi check karna add executable ke niche target_links mai catkin_LIBRARIES hai*/
    string zonestr = UTMUPS::EncodeZone(zone, northp); 
    /*double northing, easting;
    char zone;
    LLtoUTM(msg->latitude, msg->longitude, northing, easting , &zone);      
    global_position.x = easting;
    global_position.y = northing;
    global_position.z = msg->altitude;*/
    //ROS_INFO_STREAM("X: "<<x<< "Y: "<<y);
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