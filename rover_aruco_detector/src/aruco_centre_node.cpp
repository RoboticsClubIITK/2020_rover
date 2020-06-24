#include <rover_aruco_detector/aruco_centre.hpp>

using namespace roboiitk::CentreDetect;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"aruco_centre_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    CentreDetect detect;

    detect.init(nh,nh_private);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        detect.run(nh_private);
        loopRate.sleep();
    }
    return 0;
}
