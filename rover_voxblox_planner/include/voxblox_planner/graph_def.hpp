#pragma once

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <memory>

namespace rover::planner {

class GraphNode {
  public:
    typedef std::shared_ptr<GraphNode> Ptr;
    GraphNode(const Eigen::Vector3d& pos, const uint& id)
        : pos_(pos)
        , id_(id) {
    }

    uint getID() {
        return id_;
    }
    std::vector<GraphNode::Ptr> getNeighbours() {
        return neighbours_;
    }
    uint getNumNeighbours() {
        return neighbours_.size();
    }
    Eigen::Vector3d getPosition() {
        return pos_;
    }

    void setPosition(const Eigen::Vector3d& point) {
        pos_ = point;
    }
    void addNeighbour(const GraphNode::Ptr& node) {
        neighbours_.push_back(node);
    }

    void deleteNeighbour(const uint& id) {
        for (auto it = neighbours_.begin(); it != neighbours_.end(); it++) {
            if (id == (*it)->getID()) {
                neighbours_.erase(it);
                return;
            }
        }
    }

  private:
    std::vector<GraphNode::Ptr> neighbours_;
    Eigen::Vector3d pos_;
    uint state_;
    uint id_;
};

typedef GraphNode::Ptr Node;
typedef std::vector<Node> Graph;

typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point;
typedef std::pair<Point, unsigned> Value;
typedef boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> RTree;

// add RTree typedefs in this file

}  // namespace rover::planner
