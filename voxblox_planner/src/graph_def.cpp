#include <voxblox_planner/graph_def.hpp>

namespace rover::planner {

GraphNode::GraphNode(const Eigen::Vector3d& position, const uint& id) {
    // create a new node
<<<<<<< HEAD
    GraphNode::id = id;
    GraphNode::position = position;
}

void GraphNode::addNeighbour(const Ptr& node) {
    // add input id as neighbour of object node
    neighbours.push_back(node);
=======
}

void GraphNode::addNeighbour(const uint& id) {
    // add input id as neighbour of object node
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

void GraphNode::deleteNeighbour(const uint& id) {
    // delete input id from neighbours of object node
<<<<<<< HEAD
    for (auto it = neighbours.begin(); it != neighbours.end(); it++) {
        if (id == (*it)->getID()) {
            neighbours.erase(it);
            return;
        }    
    }      

=======
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

void GraphNode::setPosition(const Eigen::Vector3d& new_pos) {
    // rewrite position of object node
<<<<<<< HEAD
    position = new_pos;
}

uint GraphNode::getID() {
    return GraphNode::id;
}

Eigen::Vector3d GraphNode::getPosition(){
    return position;
} 
std::vector<GraphNode::Ptr> GraphNode::getNeighbour() {
    return neighbours;
=======
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

}  // namespace rover::planner
