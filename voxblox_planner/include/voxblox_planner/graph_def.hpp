#pragma once

<<<<<<< HEAD
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace rover::planner {

struct GraphNode {
    typedef std::shared_ptr<struct GraphNode> Ptr;
    uint id;
    Eigen::Vector3d position;
    std::vector<GraphNode::Ptr> neighbours;
    uint state;
=======
#include <Eigen/Core>
#include <memory>

namespace rover::planner {

typedef struct GraphNode {
    typedef std::shared_ptr<struct GraphNode> Ptr;
    uint id;
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
    // add members as explained

    GraphNode(const Eigen::Vector3d& pos, const uint& id);

<<<<<<< HEAD
    void addNeighbour(const Ptr& node);
    void deleteNeighbour(const uint& id);
    void setPosition(const Eigen::Vector3d& new_pos);
    uint getID();
    Eigen::Vector3d getPosition();
    std::vector<GraphNode::Ptr> getNeighbour();
    
};

// add RTree typedefs in this file

typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point;
typedef std::pair<Point, unsigned> Value;
typedef boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> RTree;
=======
    void addNeighbour(const uint& id);
    void deleteNeighbour(const uint& id);
    void setPosition(const Eigen::Vector3d& new_pos);
} Node;

// add RTree typedefs in this file

>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}  // namespace rover::planner
