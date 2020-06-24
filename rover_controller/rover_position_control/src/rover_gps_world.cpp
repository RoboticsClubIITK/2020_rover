#include<rover_position_control/rover_gps.hpp>

namespace roboiitk::gps_world{
    void rover_gps::init(ros::NodeHandle& nh,ros::NodeHandle& nh_private){
        gps_sub=nh.subscribe("fix", 1, &rover_gps::gpsCb, this);
        cmd_gps_sub=nh.subscribe("rover/cmd_gps", 1, &rover_gps::cmd_gpsCb, this);
        pose_sub=nh.subscribe("rover/odom", 1, &rover_gps::poseCb, this);
        cmd_pose_pub=nh.advertise<geometry_msgs::Pose>("rover/cmd_pose", 1);
        odom_pub=nh.advertise<geometry_msgs::Pose>("rover/odom_gps",1);
    }
    void rover_gps::cmd_gpsCb(const sensor_msgs::NavSatFix& cmd_gps_msg){
        //int zone;bool northp;
        double lat,lon,s12,azi1,azi2,rad;
        lat= cmd_gps_msg.latitude;
        lon= cmd_gps_msg.longitude;
        geod.Inverse(orig_lat, origin_lon, lat, lon, s12,azi1,azi2);
        rad= (PI/180)*azi1;
        cmd_x= cos(azi1)*s12;cmd_y= sin(azi1)*s12;
        ROS_INFO("cmd_x: %lf cmd_y: %lf",cmd_x,cmd_y);
        cmd_msg.position.x=cmd_x;//x_c+(cmd_x - odom_x);
        cmd_msg.position.y=cmd_y;//y_c+(cmd_y - odom_y);
        cmd_pose_pub.publish(cmd_msg);
        odom_msg.position.x=odom_x;
        odom_msg.position.y=odom_y;
        odom_pub.publish(odom_msg);
        //GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, des_utm_x, des_utm_y);
        //ROS_INFO("cmd_x: %lf cmd_y: %lf",des_utm_x,des_utm_y);
    }
    void rover_gps::gpsCb(const sensor_msgs::NavSatFix& gps_msg){
        //int zone;bool northp;
        double lat,lon,s12,azi1,azi2,rad;
        lat= gps_msg.latitude;
        lon= gps_msg.longitude;
        geod.Inverse(orig_lat, origin_lon, lat, lon, s12,azi1,azi2);
        rad= (PI/180)*azi1;
        odom_x= cos(azi1)*s12;odom_y= sin(azi1)*s12;
        ROS_INFO("odom_x: %lf odom_y: %lf",odom_x,odom_y);
        //GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, curr_utm_x, curr_utm_y);
        //ROS_INFO("odom_x: %lf odom_y: %lf",curr_utm_x,curr_utm_y);
    }
    void rover_gps::poseCb(const nav_msgs::Odometry& odom_msg){
        x_c=odom_msg.pose.pose.position.x;
        y_c=odom_msg.pose.pose.position.y;
    }
    void rover_gps::run(){
    }
}