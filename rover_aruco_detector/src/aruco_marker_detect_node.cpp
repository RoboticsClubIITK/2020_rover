#include <rover_aruco_detector/aruco_marker_detect.hpp>

using namespace roboiitk::ArucoDetect;

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_centre_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ArucoDetect aruco_detect;

    aruco_detect.init(nh, nh_private);

    ros::Rate loopRate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
