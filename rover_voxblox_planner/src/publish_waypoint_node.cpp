#include <voxblox_planner/publish_waypoint.hpp>

using namespace rover::planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_waypoint_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    publish_waypoint publish_waypoint;

    publish_waypoint.init(nh, nh_private);

    ros::Rate loopRate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
        publish_waypoint.run();
    }
    return 0;
}
