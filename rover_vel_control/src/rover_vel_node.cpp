#include <rover_vel_control/vel_cntrl.hpp>

using namespace roboiitk::vel_control;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rover_vel_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    rover rover;

    rover.init(nh,nh_private);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        rover.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;

}