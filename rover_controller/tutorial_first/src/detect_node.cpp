#include <rover/detect_box.hpp>

using namespace roboiitk::detect;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"detect_box_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    box_detect detect;

    detect.init(nh,nh_private);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        detect.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
