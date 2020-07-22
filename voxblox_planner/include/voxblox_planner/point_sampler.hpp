#pragma once

<<<<<<< HEAD
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
=======
#include <Eigen/Core>
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
#include <random>

namespace rover::planner {
class PointSampler {
  public:
    PointSampler();
    void init(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
<<<<<<< HEAD
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
=======
    Eigen::Vector3d generateSample();

  private:
    // define random number generator
    // store sampler region dimensions
    // store frame transformations
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
};
}  // namespace rover::planner
