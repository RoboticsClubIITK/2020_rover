#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <random>
#include <ros/ros.h>

namespace rover::planner {

class PointSampler {
  public:
    void init(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    double random() {
        return -1 + 2.0 * (double(rand()) / RAND_MAX);
    }
    Eigen::Vector3d generateSample();
    double getWidth() {
        return smpl_(1);
    }

  private:
    // define random number generator
    Eigen::Matrix3d rot_mat_;
    Eigen::Vector3d translation_, smpl_;
    // store sampler region dimensions
    // store frame transformations
};

}  // namespace rover::planner
