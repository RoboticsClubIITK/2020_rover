#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <GeographicLib/UTMUPS.hpp>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <stdlib.h>
#include <string>
using namespace GeographicLib;
using namespace std;

int zone;
bool northp;
double x, y, g, k;
void saw(const sensor_msgs::NavSatFixConstPtr& msg) {
    UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, x, y, g, k);  // x=x coodinate of rover ,y=y coordinate of rover
    std::string zonestr = UTMUPS::EncodeZone(zone, northp);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_generation");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/fix", 100, saw);
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/rover/target", 100);

    ros::Rate rate(100);

    int zone_t;
    bool northp_t;
    double x_t, y_t, g_t, k_t;
    double latitude = 30, longitude = 30;
    cout << "Entre target's latitude: " << endl;  // taking inputs for coordinates
    cin >> latitude;
    cout << "Entre target's longitude: " << endl;
    cin >> longitude;
    UTMUPS::Forward(latitude, longitude, zone_t, northp_t, x_t, y_t, g_t, k_t);
    std::string zonestr_t = UTMUPS::EncodeZone(zone_t, northp_t);

    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Point goal;
        goal.x = x_t;
        goal.y = y_t;
        pub.publish(goal);
        rate.sleep();
    }
    return 0;
}
