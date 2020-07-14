#pragma once

#include <Eigen/Core>
#include <mav_planning_msgs/PlannerService.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <voxblox_planner/graph_def.hpp>
#include <voxblox_planner/point_sampler.hpp>
#include <voxblox_ros/esdf_server.h>

namespace rover::planner {

class PlannerNode {
  public:
    PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  private:
    void plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp);
    void publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

    double getMapDistance(const Eigen::Vector3d& point);

    void createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    void findPath(const uint& start_index, const uint& end_index);

    void shortenPath();
    bool isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

    std::vector<Eigen::Vector3d> curr_path_;
    std::vector<Eigen::Vector3d> short_path_;

    std::vector<GraphNode::Ptr> graph_;

    voxblox::EsdfServer server_;
    PointSampler sampler_;

    double robot_radius_;
    double voxel_size_;
    uint num_neighbours_;  // k for graph

    ros::Publisher path_pub_;

    ros::ServiceServer planner_server_;
    ros::ServiceServer publish_server_;
};

}  // namespace rover::planner
