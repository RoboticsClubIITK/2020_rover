#include <voxblox_planner/point_sampler.hpp>

namespace rover::planner {

PointSampler::PointSampler() {
    // initialize your random number generator here
    //std::mt19937 generator(device());
    //std::uniform_real_distribution<float> distribution(-1,1);
    
    //for(int i =0;i<3;i++){
    ///    random[i] = distribution(generator);
    //}   

    engine_ = std::default_random_engine(rd_());
    dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    
    //random.Random() //= Eigen::Vector3d::Random(); 
}

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // define your sampler frame and calculate transformations here
    region_ = Eigen::Vector3d(0.5, 1.0, 0.5);
    region_(0) += 0.5 * (end - start).norm();
    translation_ = 0.5 * (start + end);

    rotation_.col(0) = (end - translation_).normalized();
    rotation_.col(1) = rotation_.col(0).cross(Eigen::Vector3d(0, 0, -1)).normalized();
    rotation_.col(2) = rotation_.col(0).cross(rotation_.col(1));
}

Eigen::Vector3d PointSampler::generateSample() {
    // generate a sample
    Eigen::Vector3d point;
    point(0) = dist_(engine_) * region_(0);
    point(1) = dist_(engine_) * region_(1);
    point(2) = dist_(engine_) * region_(2);

    return (rotation_ * point + translation_);
}
void PointSampler::expandRegion(const double& size) {
    region_(1) += size;
}

}  // namespace rover::planner