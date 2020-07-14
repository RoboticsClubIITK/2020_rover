#include <voxblox_planner/voxblox_planner.hpp>

namespace rover::planner {

PlannerNode::PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : sampler_()
    , server_(nh, nh_private) {
    // create all your pubs, subs, servers etc using the input nodehandles
}

void PlannerNode::plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp) {
    Eigen::Vector3d start, end;
    // get start and end point from request

    // create graph function
    // find path function
    // shorten path function
}

void PlannerNode::publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
    // publish generated path
}

void PlannerNode::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // clear existing graph_
    // push back start and end nodes into graph
    // initialize the sampler and use it to get samples
    // create nodes from sampled positions and push back into graph
    // insert all node positions and ids into rtree
    // query rtree for k neighbours of each node
    // insert neighbours and return
}

void PlannerNode::findPath(const uint& start_index, const uint& end_index) {
    // clear existing curr_path_
    // implement A*, keep track of parents in a separate vector
    // once end node is reached, backtrack and push all nodes into path
}

bool PlannerNode::isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // check for collisions in the line joining start and end
    // using getMapDistance and robot_radius
    // in increments of voxel size
}

void PlannerNode::shortenPath() {
    // clear existing short_path_
    // use the collision checker on A* generated path
}

double PlannerNode::getMapDistance(const Eigen::Vector3d& position) {
    // call the server's method and return the distance at input point.
}

}  // namespace rover::planner
