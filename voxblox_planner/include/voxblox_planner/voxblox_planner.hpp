#pragma once

<<<<<<< HEAD
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_msgs/conversions.h>
//#include <voxblox_planning_common/path_shortening.h>
#include <voxblox_planner/path_shortening.h>
#include <string>
#include <voxblox_planner/PlannerService.h>
=======
#include <Eigen/Core>
#include <mav_planning_msgs/PlannerService.h>
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <voxblox_planner/graph_def.hpp>
#include <voxblox_planner/point_sampler.hpp>
<<<<<<< HEAD
#include "voxblox_ros/esdf_server.h"
#include <geometry_msgs/PoseArray.h>
=======
#include <voxblox_ros/esdf_server.h>
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf

namespace rover::planner {

class PlannerNode {
  public:
    PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  private:
<<<<<<< HEAD
    bool plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp);
    bool publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
=======
    void plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp);
    void publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf

    double getMapDistance(const Eigen::Vector3d& point);

    void createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    void findPath(const uint& start_index, const uint& end_index);

    void shortenPath();
    bool isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
<<<<<<< HEAD
    void findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map);
=======
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf

    std::vector<Eigen::Vector3d> curr_path_;
    std::vector<Eigen::Vector3d> short_path_;

    std::vector<GraphNode::Ptr> graph_;
<<<<<<< HEAD
    RTree tree_;
    voxblox::EsdfServer server_;
    PointSampler sampler_;


    //mav_planning::EsdfPathShortener path_shortener_.setEsdfLayer(server_.getEsdfMapPtr()->getEsdfLayerPtr());
    ///mav_msgs::EigenTrajectoryPointVector last_waypoints_;
    uint p_sample_;
    double robot_radius_;
    double voxel_size_ = double(server_.getEsdfMapPtr()->voxel_size());
    uint num_neighbours_;  // k for graph

    //std::string frame_id_;
=======

    voxblox::EsdfServer server_;
    PointSampler sampler_;

    double robot_radius_;
    double voxel_size_;
    uint num_neighbours_;  // k for graph

>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
    ros::Publisher path_pub_;

    ros::ServiceServer planner_server_;
    ros::ServiceServer publish_server_;
};

}  // namespace rover::planner
