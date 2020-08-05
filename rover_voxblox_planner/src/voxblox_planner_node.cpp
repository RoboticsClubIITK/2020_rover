#include <voxblox_planner/voxblox_planner.hpp>

using namespace rover::planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    PlannerNode planner(nh, nh_private);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
