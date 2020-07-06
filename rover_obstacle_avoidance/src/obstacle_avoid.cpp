#include<rover_obstacle_avoidance/obstacle_avoid.hpp> 

namespace roboiitk::obstacle_avoid{
    void obstacleAvoid::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
        laser_sub=nh.subscribe("/rover/scan", 10, &obstacleAvoid::laserCb,this);
        heading_sub=nh.subscribe("rover/heading", 10, &obstacleAvoid::headingCb, this);
        pose_sub=nh.subscribe("rover/odom", 10, &obstacleAvoid::poseCb, this);
        pub_vel=nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        nh_private.getParam("obstacle_min_distance_threshold",obstacle_min_distance_threshold);
        nh_private.getParam("angle_threshold_low",angle_threshold_low);
        nh_private.getParam("angle_threshold_high",angle_threshold_high);
        nh_private.getParam("distance_threshold",distance_threshold);
        nh_private.getParam("heading_threshold",heading_threshold);
        setGoal();
    }
    void obstacleAvoid::setGoal(){
        goal.x=-95.0;
        goal.y=40.0;
    }
    void obstacleAvoid::headingCb(const geometry_msgs::Vector3& head_msg){
        heading=head_msg.z;
    }
    void obstacleAvoid::poseCb(const nav_msgs::Odometry& odom_msg){
        x_c=odom_msg.pose.pose.position.x;
        y_c=odom_msg.pose.pose.position.y;
    }
    void obstacleAvoid::laserCb(const sensor_msgs::LaserScan& laser_msg){
        min_distance = (laser_msg.ranges)[0];

        for(int j=0;j<360;j++){
            range[j] = (laser_msg.ranges)[j];
            if(range[j]<=min_distance)
            {
                min_distance=range[j];
                min_distance_angle=j/2;
            }
        }
    }
    void obstacleAvoid::moveForward(){
        speed_msg.linear.x = 60.0;
        speed_msg.angular.z = 0;
    }
    void obstacleAvoid::moveRight(){
        speed_msg.linear.x=0.5;
        speed_msg.angular.z=-20.0;
    }
    void obstacleAvoid::moveLeft(){
        speed_msg.linear.x=0.5;
        speed_msg.angular.z=20.0;
    }
    void obstacleAvoid::stop(){
        speed_msg.linear.x =0;
        speed_msg.angular.z=0;
    }  
    void obstacleAvoid::run(){            
        dist = sqrt((goal.x -x_c)*(goal.x -x_c)+ (goal.y -y_c)*(goal.y -y_c));
        angle = std::atan(((goal.y -y_c))/(goal.x -x_c));
        tar_heading =  heading - angle ;
        if(min_distance<=obstacle_min_distance_threshold)
        {
            if((2*min_distance_angle)<angle_threshold_low|| (2*min_distance_angle)>angle_threshold_high){
                moveForward();
            }
            else {
                if(min_distance_angle<90){
                    moveRight();
                }
                else{
                    moveLeft();
                }     
            }
        }
        else
        {
            if (tar_heading > heading_threshold){
                moveLeft();
            }
            else if(tar_heading < -(heading_threshold)){
                moveRight();
            }
            else{
                moveForward();
            }
            if(dist <distance_threshold){
                stop();
            }
        }
        pub_vel.publish(speed_msg);
    }
}