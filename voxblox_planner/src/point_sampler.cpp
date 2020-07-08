#include <voxblox_planner/point_sampler.hpp>

namespace rover::planner {

PointSampler::PointSampler() {
    // initialize your random number generator here
}

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // define your sampler frame and calculate transformations here
}

Eigen::Vector3d PointSampler::generateSample() {
    // generate a sample
}

}  // namespace rover::planner
