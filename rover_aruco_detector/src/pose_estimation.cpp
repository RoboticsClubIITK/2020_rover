#include <rover_aruco_detector/pose_estimation.hpp>

namespace roboiitk::pose_estimation {
    void PoseEstimator::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private){
        centre_coord_sub_ = nh.subscribe("aruco_box_centre", 10, &PoseEstimator::centreCallback, this);
        odom_sub_ = nh.subscribe("/rover/odom", 10, &PoseEstimator::odomCallback, this);

        glob_coord_pub_ = nh_private.advertise<geometry_msgs::Point>("estimated_coord", 10);
        img_vec_ = Eigen::Vector3d(0, 0, 1);
        std::vector<double> temp_list;
        nh_private.getParam("camera_matrix", temp_list);
        setCamMatrix(temp_list);

        temp_list.clear();
        nh_private.getParam("cam_to_Rover_rot", temp_list);
        setCamToRoverMatrix(temp_list);

        temp_list.clear();
        nh_private.getParam("t_cam", temp_list);
        setTCamMatrix(temp_list);
    }
    void PoseEstimator::getDistance(const float& dist) {
        // clang-format off
        scale_up_ << dist, 0, 0,
                    0, dist, 0,
                    0, 0, dist;
        // clang-format on
    }

    void PoseEstimator::setImgVec(const float& x, const float& y) {
        img_vec_(0) = x;
        img_vec_(1) = y;
    }

    void PoseEstimator::setQuaternion(const nav_msgs::Odometry& odom) {
        geometry_msgs::Quaternion odom_quat = odom.pose.pose.orientation;
        tf::Quaternion quat = tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
        Eigen::Quaterniond eigen_quat = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
        Rover_to_glob_ = eigen_quat.normalized().toRotationMatrix();
    }

    void PoseEstimator::CamToRover() { Rover_coord_ = cam_to_Rover_ * scale_up_ * cam_matrix_.inverse() * img_vec_ + t_cam_; }

    void PoseEstimator::RoverToGlob(const nav_msgs::Odometry& odom) {
        glob_coord_ = Rover_to_glob_ * Rover_coord_;

        glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
        glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
        glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
    }
    void PoseEstimator::run() {
        if ((centre_coord_.x == -1) || (centre_coord_.y == -1)) {
            glob_coord_pub_.publish(global_coord_);
            return;
        }

        glob_coord_ = calculateGlobCoord(centre_coord_.x, centre_coord_.y, centre_coord_.z);
        global_coord_.x = glob_coord_(0);
        global_coord_.y = glob_coord_(1);
        global_coord_.z = 0;//glob_coord_(2);
        glob_coord_pub_.publish(global_coord_);
    }

    Eigen::Vector3d PoseEstimator::calculateGlobCoord(const double& img_x, const double& img_y, const double& dist) {
        getDistance(dist);
        setImgVec(img_x, img_y);
        CamToRover();
        setQuaternion(odom_);
        RoverToGlob(odom_);
        return getGlobCoord();
    }
}