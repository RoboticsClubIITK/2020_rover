#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int16MultiArray.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>


geometry_msgs::Pose aruco_pose;
geometry_msgs::Point global_coord_;
nav_msgs::Odometry odom;
Eigen::Matrix3f camToRover, RoverToCam;
Eigen::Vector3f tCam;
int imageID; int types[3] = {0,0,0};
double maxCentreError;
int height, width;

void poseCallback(const geometry_msgs::Pose& msg){aruco_pose = msg;}
void odomCallback(const nav_msgs::Odometry& msg){odom = msg;}

Eigen::Vector3f convertPose(geometry_msgs::Pose msg)
{
    Eigen::Matrix3f RoverToGlob;

    tf::Quaternion q1(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    Eigen::Quaternionf quat = Eigen::Quaternionf(q1.w(), q1.x(), q1.y(), q1.z());
    RoverToGlob = quat.toRotationMatrix();
    
    Eigen::Vector3f camCoord(msg.position.x, msg.position.y, msg.position.z - odom.pose.pose.position.z);
    Eigen::Vector3f RoverCoord = camToRover*camCoord + tCam;
    Eigen::Vector3f globCoord = RoverToGlob*RoverCoord;

    globCoord(0) = globCoord(0) + odom.pose.pose.position.x;
    globCoord(1) = globCoord(1) + odom.pose.pose.position.y;
    globCoord(2) = globCoord(2) + odom.pose.pose.position.z;

    return globCoord;   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_pose");
    ros::NodeHandle nh, ph("~");

    /*ph.getParam("marker1", types[0]);
    ph.getParam("marker2", types[1]);
    ph.getParam("marker3", types[2]);*/

    //ph.getParam("maxCentreError", maxCentreError);
    //ph.getParam("width", width);
    //ph.getParam("height", height);

    std::vector<double> tempList;
    ph.getParam("camera/translation", tempList);
    for (int i = 0; i < 3; i++)
    {
        tCam(i) = tempList[i];
    }

    int tempIdx = 0; tempList.clear();
    ph.getParam("camera/rotation", tempList);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            RoverToCam(i,j) = tempList[tempIdx++];
        }
    }
    camToRover = RoverToCam.inverse();

    ros::Subscriber pose_sub = nh.subscribe("pose1", 1, poseCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);

    ros::Publisher aruco_pose_pub = nh.advertise<geometry_msgs::Point >("aruco_poses", 1);

    ros::Rate loopRate(10);

    Eigen::Vector3f aruco_cord;

    while(ros::ok())
    {
        ros::spinOnce();
        aruco_cord=convertPose(aruco_pose);
        global_coord_.x = aruco_cord(0);
        global_coord_.y = aruco_cord(1);
        global_coord_.z = aruco_cord(2);
        aruco_pose_pub.publish(global_coord_);
        loopRate.sleep();
    }

    return 0;
}
