#pragma once

#include <Eigen/Core>
#include <random>

namespace rover::planner {
class PointSampler {
  public:
    PointSampler();
    void init(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    Eigen::Vector3d generateSample();

  private:
    // define random number generator
    // store sampler region dimensions
    // store frame transformations
};
}  // namespace rover::planner
