#include<rover_position_control/rover_pose_control.hpp>

using namespace roboiitk::pose_control;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rover_position_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    rover_pose rover_pose;

    rover_pose.init(nh,nh_private);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;

}