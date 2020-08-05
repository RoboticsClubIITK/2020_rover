#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace roboiitk::pose_estimation {

class PoseEstimator {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();

    void centreCallback(const geometry_msgs::Point& msg) {
        centre_coord_ = msg;
    };
    void odomCallback(const nav_msgs::Odometry& msg) {
        odom_ = msg;
    };
    void getDistance(const float& dist);
    Eigen::Vector3d getGlobCoord() {
        return glob_coord_;
    };

    void setCamToRoverMatrix(const std::vector<double>& mat) {
        cam_to_Rover_ = Eigen::Matrix3d(mat.data()).transpose();
    };
    void setCamMatrix(const std::vector<double>& mat) {
        cam_matrix_ = Eigen::Matrix3d(mat.data()).transpose();
    };
    void setTCamMatrix(const std::vector<double>& mat) {
        t_cam_ = Eigen::Vector3d(mat.data());
    };
    void setImgVec(const float& x, const float& y);
    void setQuaternion(const nav_msgs::Odometry& odom);

    void CamToRover();
    void RoverToGlob(const nav_msgs::Odometry& odom);

  private:
    Eigen::Matrix3d scale_up_;
    Eigen::Matrix3d cam_matrix_;
    Eigen::Matrix3d cam_to_Rover_;
    Eigen::Matrix3d Rover_to_glob_;

    Eigen::Vector3d img_vec_;
    Eigen::Vector3d t_cam_;
    Eigen::Vector3d Rover_coord_;
    Eigen::Vector3d glob_coord_;
    Eigen::Vector3d calculateGlobCoord(const double& img_x, const double& img_y, const double& dist);

    geometry_msgs::Point centre_coord_;
    geometry_msgs::Point global_coord_;

    nav_msgs::Odometry odom_;

    ros::Subscriber centre_coord_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher glob_coord_pub_;
};

}  // namespace roboiitk::pose_estimation
