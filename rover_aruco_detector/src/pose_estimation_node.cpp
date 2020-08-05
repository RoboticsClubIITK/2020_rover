#include <rover_aruco_detector/pose_estimation.hpp>

using namespace roboiitk::pose_estimation;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_estimator_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    PoseEstimator pose_est;

    pose_est.init(nh, nh_private);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
        pose_est.run();
    }

    return 0;
}
