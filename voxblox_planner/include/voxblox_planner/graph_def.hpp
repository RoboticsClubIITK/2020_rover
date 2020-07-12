#pragma once

#include <Eigen/Core>
#include <memory>

namespace rover::planner {

typedef struct GraphNode {
    typedef std::shared_ptr<struct GraphNode> Ptr;
    uint id;
    // add members as explained

    GraphNode(const Eigen::Vector3d& pos, const uint& id);

    void addNeighbour(const uint& id);
    void deleteNeighbour(const uint& id);
    void setPosition(const Eigen::Vector3d& new_pos);
} Node;

// add RTree typedefs in this file

}  // namespace rover::planner
