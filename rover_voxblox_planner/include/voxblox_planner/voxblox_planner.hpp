#pragma once

#include <Eigen/Core>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_msgs/conversions.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <voxblox_planner/graph_def.hpp>
#include <voxblox_planner/point_sampler.hpp>
#include <voxblox_ros/esdf_server.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

namespace rover::planner {

typedef std::vector<Eigen::Vector3d> Path;

class PlannerNode {
  public:
    PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  private:
    bool plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp);
    bool publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

    double getMapDistance(const Eigen::Vector3d& point);

    void createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    void findPath(const uint& start_index, const uint& end_index);

    void shortenPath();
    void findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map);
    bool isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

    Path raw_path_;
    Path curr_path_;
    Path short_path_;

    Graph graph_;
    RTree tree_;

    voxblox::EsdfServer server_;
    PointSampler sampler_;

    double robot_radius_;
    double voxel_size_;
    int num_neighbours_;  // k for graph
    int p_sample_;

    ros::Publisher path_pub_;

    ros::ServiceServer planner_server_;
    ros::ServiceServer publish_server_;
};

}  // namespace rover::planner
