#include <voxblox_planner/graph_def.hpp>

namespace rover::planner {

GraphNode::GraphNode(const Eigen::Vector3d& position, const uint& id) {
    // create a new node
}

void GraphNode::addNeighbour(const uint& id) {
    // add input id as neighbour of object node
}

void GraphNode::deleteNeighbour(const uint& id) {
    // delete input id from neighbours of object node
}

void GraphNode::setPosition(const Eigen::Vector3d& new_pos) {
    // rewrite position of object node
}

}  // namespace rover::planner
