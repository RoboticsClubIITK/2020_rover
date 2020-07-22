<<<<<<< HEAD
#include "voxblox_planner/voxblox_planner.hpp"

namespace rover::planner {

bool PlannerNode::plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp) {
    // get start and end point from request
    Eigen::Vector3d start(req.start_pose.pose.position.x, req.start_pose.pose.position.y,req.start_pose.pose.position.z);
    Eigen::Vector3d end(req.goal_pose.pose.position.x, req.start_pose.pose.position.y, req.start_pose.pose.position.z);
    // create graph function
    PlannerNode::createGraph(start,end);
    // find path function
    PlannerNode::findPath(0,1);
    // shorten path function
    PlannerNode::shortenPath();
    //mav_msgs::EigenTrajectoryPointVector short_path;
    //path_shortener_.shortenPath();
    //last_waypoints_ = short_path;
    return true;
}

bool PlannerNode::publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
    // publish generated path
    geometry_msgs::PoseArray pose_array;
    pose_array.poses.reserve(short_path_.size());
    //for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
        //geometry_msgs::PoseStamped pose_stamped;
        //mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
        /////pose_array.poses.push_back(pose_stamped.pose);
    //}
    //pose_array.header.frame_id = frame_id_;
    path_pub_.publish(pose_array);
    //path_pub_.publish();
    return true;
    
=======
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
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

void PlannerNode::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // clear existing graph_
<<<<<<< HEAD
    graph_.clear();
    // push back start and end nodes into graph
    sampler_.init(start, end);
    // initialize the sampler and use it to get samples
    double p_sample = p_sample_;
    double robot_radius = robot_radius_;
    // create nodes from sampled positions and push back into graph

    uint max_samples = p_sample * (start - end).norm() * sampler_.getWidth() / voxel_size_;
    uint num_sample = 0;

    graph_.push_back(GraphNode::Ptr(new GraphNode(start, 0)));
    graph_.push_back(GraphNode::Ptr(new GraphNode(end, 1)));

    // insert all node positions and ids into rtree
    uint node_id = 2;
    while (num_sample++ < max_samples) {
        Eigen::Vector3d sample = sampler_.generateSample();
        double distance = 0.0;
        distance = getMapDistance(sample);
        if ( distance >= robot_radius) {
            graph_.push_back(GraphNode::Ptr(new GraphNode(sample, node_id++)));
        }
    }
    // query rtree for k neighbours of each node
    tree_.clear();
    for (auto& node : graph_) {
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.insert(std::make_pair(pt, node->getID()));
    }
    // insert neighbours and return
    for (auto& node : graph_) {
        std::vector<Value> neighbours;
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.query(boost::geometry::index::nearest(pt, (unsigned) num_neighbours_ + 1), std::back_inserter(neighbours));
        for (auto& neighbour : neighbours) {
            if (neighbour.second != node->getID()) {
                node->addNeighbour(graph_[neighbour.second]);
            }
        }
    }
=======
    // push back start and end nodes into graph
    // initialize the sampler and use it to get samples
    // create nodes from sampled positions and push back into graph
    // insert all node positions and ids into rtree
    // query rtree for k neighbours of each node
    // insert neighbours and return
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

void PlannerNode::findPath(const uint& start_index, const uint& end_index) {
    // clear existing curr_path_
<<<<<<< HEAD
    if (graph_.empty()) {
        return;
    }
    curr_path_.clear();
    Eigen::Vector3d end_pos = graph_[end_index]->getPosition();
    typedef std::pair<double, uint> f_score_map;
    // implement A*, keep track of parents in a separate vector
    // once end node is reached, backtrack and push all nodes into path
    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(graph_.size(), DBL_MAX);
    std::vector<uint> parent(graph_.size(), INT_MAX);

    open_set.push(std::make_pair((end_pos - graph_[start_index]->getPosition()).norm(), start_index));
    g_score[start_index] = 0.0;

    while (!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = graph_[curr_index]->getPosition();
        open_set.pop();

        if (curr_index == end_index) {
            std::vector<Eigen::Vector3d> curr_path;
            while (parent[curr_index] != INT_MAX) {
                curr_path.push_back(graph_[curr_index]->getPosition());
                curr_index = parent[curr_index];
            }

            curr_path.push_back(graph_[start_index]->getPosition());
            std::reverse(curr_path.begin(), curr_path.end());
            curr_path_ = curr_path;
            return;
        }

        for (auto & neigh : graph_[curr_index]->getNeighbour()) {
            uint neigh_index = neigh->getID();
            Eigen::Vector3d neigh_pos = neigh->getPosition();

            double score = g_score[curr_index] + (neigh_pos - curr_pos).norm();
            if (score < g_score[neigh_index]) {
                g_score[neigh_index] = score;
                parent[neigh_index] = curr_index;
                open_set.push(std::make_pair(score + (end_pos - neigh_pos).norm(), neigh_index));
            }
        }
    }

=======
    // implement A*, keep track of parents in a separate vector
    // once end node is reached, backtrack and push all nodes into path
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

bool PlannerNode::isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // check for collisions in the line joining start and end
    // using getMapDistance and robot_radius
    // in increments of voxel size
<<<<<<< HEAD
    double distance = (end - start).norm();
    if (distance < voxel_size_) {
        return false;
    }
    Eigen::Vector3d direction = (end - start).normalized();
    Eigen::Vector3d curr_pos = start;
    double cum_dist = 0.0;

    while (cum_dist <= distance) {
        voxblox::EsdfVoxel* esdf_voxel_ptr = server_.getEsdfMapPtr()->getEsdfLayerPtr()->getVoxelPtrByCoordinates(curr_pos.cast<voxblox::FloatingPoint>());
        if (esdf_voxel_ptr == nullptr) {
            return true;
        }
        if (esdf_voxel_ptr->distance < robot_radius_) {
            return true;
        }

        double step_size = std::max(voxel_size_, esdf_voxel_ptr->distance - robot_radius_);

        curr_pos += direction * step_size;
        cum_dist += step_size;
    }
    return false;

=======
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

void PlannerNode::shortenPath() {
    // clear existing short_path_
<<<<<<< HEAD
    if (short_path_.empty()) {
        return;
    }
    short_path_.clear();
    std::vector<bool> retain(short_path_.size(), false);

    findMaximalIndices(0, short_path_.size() - 1, &retain);
    for (uint i = 0; i < short_path_.size(); i++) {
        if (retain[i])
            short_path_.push_back(short_path_[i]);
    }
    // use the collision checker on A* generated path
}
void PlannerNode::findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map) {
    if (start >= end) {
        return;
    }
    if (!isLineInCollision(short_path_[start], short_path_[end])) {
        (*map)[start] = (*map)[end] = true;
        return;
    } else {
        if (start == end) {
            return;
        }
        uint centre = (start + end) / 2;
        findMaximalIndices(start, centre, map);
        findMaximalIndices(centre, end, map);
    }
}
double PlannerNode::getMapDistance(const Eigen::Vector3d& position) {
    // call the server's method and return the distance at input point.
    if (!server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                        &distance)) {
    return 0.0;
    }
    return distance;
}

PlannerNode::PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : sampler_()
    , server_(nh, nh_private) {
    // create all your pubs, subs, servers etc using the input nodehandles
    path_pub_ = nh.advertise<geometry_msgs::PoseArray>("/seqOfWayPoints", 100);
    PlannerNode::planner_server_ = nh_private.advertiseService("plan",plannerServiceCallback,this);
    //publish_server_ = nh_private.advertiseService("publish_path", &PlannerNode::publishPathCallback);
=======
    // use the collision checker on A* generated path
}

double PlannerNode::getMapDistance(const Eigen::Vector3d& position) {
    // call the server's method and return the distance at input point.
>>>>>>> 192e2ac5b02d5d3ed07c7ccc4667e02b1afdfdcf
}

}  // namespace rover::planner
