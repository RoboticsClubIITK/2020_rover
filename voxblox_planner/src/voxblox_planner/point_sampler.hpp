#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <random>

namespace rover::planner {
class PointSampler {
  public:
    PointSampler();
    void init(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    void expandRegion(const double& size);
    Eigen::Vector3d generateSample();
    double getWidth() {
      return region_(1);
    }


  private:
    // define random number generator
    //Eigen::Vector3d random;
    std::random_device rd_;
    //double random[3] ={};
    std::default_random_engine engine_;
    std::uniform_real_distribution<double> dist_;    
    // store sampler region dimensions
    Eigen::Vector3d region_;
    // store frame transformations
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d translation_;
};
}  // namespace rover::planner
