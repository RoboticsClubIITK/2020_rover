#include<rover_position_control/rover_gps.hpp>

namespace roboiitk::gps_world{
    void rover_gps::init(ros::NodeHandle& nh,ros::NodeHandle& nh_private){
        gps_sub=nh.subscribe("rover/gps", 1, &rover_gps::gpsCb, this);
        cmd_pose_pub=nh.advertise<geometry_msgs::Pose>("rover/cmd_pose", 1);
    }
    void rover_gps::gpsCb(const sensor_msgs::NavSatFix& gps_msg){
        double lat,lon,s12,azi1,azi2,rad;
        lat= gps_msg.latitude;
        lon= gps_msg.longitude;
        geod.Inverse(orig_lat, origin_lon, lat, lon, s12,azi1,azi2);
        rad= (PI/180)*azi1;
        ROS_INFO("distance: %lf angle: %lf",s12,rad);
        cmd_x= cos(azi1)*s12;cmd_y= sin(azi1)*s12;
    }
    void rover_gps::run(){
        cmd_msg.position.x=cmd_x;
        cmd_msg.position.y=cmd_y;
        ROS_INFO("cmd_x: %lf cmd_y: %lf",cmd_x,cmd_y);
        cmd_pose_pub.publish(cmd_msg);
    }
}