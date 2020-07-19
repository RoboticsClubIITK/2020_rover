#pragma once

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
    // add members as explained

    GraphNode(const Eigen::Vector3d& pos, const uint& id);

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
}  // namespace rover::planner
