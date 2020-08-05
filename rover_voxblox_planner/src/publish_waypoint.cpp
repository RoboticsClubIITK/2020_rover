#include<voxblox_planner/publish_waypoint.hpp>

namespace rover::planner {
    void publish_waypoint::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
        odom_sub_=nh.subscribe("rover/odom", 1, &publish_waypoint::odomCb, this);
        path_sub_=nh.subscribe("firefly/waypoint_list", 1, &publish_waypoint::pathCb, this);
        pub_waypoint_=nh.advertise<geometry_msgs::Pose>("rover/cmd_pose", 1);
    }
    void publish_waypoint::odomCb(const nav_msgs::Odometry& odom_msg){
        odom_msg_= odom_msg;
        if(id!=-1){
            ROS_INFO("Publishing command");
            double x=path_msg_.poses[id].position.x;
            double y=path_msg_.poses[id].position.y;
            double x_=odom_msg_.pose.pose.position.x;
            double y_=odom_msg_.pose.pose.position.y;
            dist_= sqrt((x-x_)*(x-x_)+(y-y_)*(y-y_));
            if(dist_<1 &&(id+1 < int(path_msg_.poses.size()))){
                id++;
                cmd_msg_.position.x=path_msg_.poses[id].position.x;
                cmd_msg_.position.y=path_msg_.poses[id].position.y;
                cmd_msg_.position.z= 0;
                ROS_INFO("Reached one Waypoint");
            }
            else{
                cmd_msg_.position.x=x;
                cmd_msg_.position.y=y;
                cmd_msg_.position.z=0;
            }
        }
    }
    void publish_waypoint::pathCb(const geometry_msgs::PoseArray& path_msg){
        path_msg_=path_msg;
        id=0;
        ROS_INFO("The path starts:");
        for(int i=0;i<int(path_msg_.poses.size());i++){
            ROS_INFO(" point %d is:(%lf,%lf)",i+1,path_msg_.poses[i].position.x,path_msg_.poses[i].position.y);
        }
    }
    void publish_waypoint::run(){
        pub_waypoint_.publish(cmd_msg_);
    }
}  // namespace rover::planner