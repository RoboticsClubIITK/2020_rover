#include<rover_obstacle_avoidance/obstacle_avoid.hpp>

using namespace roboiitk::obstacle_avoid;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rover_obstacle_avoidance_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    obstacleAvoid obstacleAvoid;

    obstacleAvoid.init(nh,nh_private);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        obstacleAvoid.run();
        loopRate.sleep();
    }
    return 0;

}