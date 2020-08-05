#include <voxblox_planner/point_sampler.hpp>

namespace rover::planner {

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // define your sampler frame and calculate transformations here
    smpl_ = Eigen::Vector3d(0.5, 1.0, 0.5);
    smpl_(0) += 0.5 * (end - start).norm();
    translation_ = 0.5 * (start + end);

    rot_mat_.col(0) = (end - translation_).normalized();
    rot_mat_.col(1) = rot_mat_.col(0).cross(Eigen::Vector3d(0, 0, -1)).normalized();
    rot_mat_.col(2) = rot_mat_.col(0).cross(rot_mat_.col(1));
}

Eigen::Vector3d PointSampler::generateSample() {
    // generate a sample
    Eigen::Vector3d point;
    point(0) = random() * smpl_(0);
    point(1) = random() * smpl_(1);
    point(2) = random() * smpl_(2);

    return (rot_mat_ * point + translation_);
}

}  // namespace rover::planner
