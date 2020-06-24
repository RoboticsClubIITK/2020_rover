#include<rover_position_control/rover_gps.hpp>

using namespace roboiitk::gps_world;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rover_gps_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    rover_gps rover_gps;

    rover_gps.init(nh,nh_private);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        rover_gps.run();
        loopRate.sleep();
    }
    return 0;

}