#include <voxblox_planner/voxblox_planner.hpp>

namespace rover::planner {

PlannerNode::PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : sampler_()
    , server_(nh, nh_private)
    , p_sample_(10) {
    planner_server_= nh_private.advertiseService("plan", &PlannerNode::plannerServiceCallback, this);
    publish_server_= nh_private.advertiseService("publish_path", &PlannerNode::publishPathCallback, this);
    path_pub_= nh.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);
    nh_private.getParam("num_neighbours", num_neighbours_);
    nh_private.getParam("robot_radius", robot_radius_);
    voxel_size_ = double(server_.getEsdfMapPtr()->voxel_size());
}

bool PlannerNode::plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& req, mav_planning_msgs::PlannerServiceResponse& resp) {
    // get start and end point from request
    ROS_INFO("planner service called");
    mav_msgs::EigenTrajectoryPoint start_pose, end_pose;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(req.start_pose, &start_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(req.goal_pose, &end_pose);
    Eigen::Vector3d start=start_pose.position_W,end=end_pose.position_W;
    ROS_INFO("start point x: %lf y: %lf  \n end point x: %lf y: %lf  ",start(0),start(1),end(0),end(1));
    createGraph(start,end);
    findPath(graph_.front()->getID(),graph_.back()->getID());
    shortenPath();
    return true;
}

bool PlannerNode::publishPathCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
    // publish generated path
    ROS_INFO("Publishing waypoints for the path.");
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pos_;
    pose_array.poses.reserve(short_path_.size());
    for(int i=0;i<int(short_path_.size());i++){
        pos_.position.x=short_path_[i](0);
        pos_.position.y=short_path_[i](1);
        pos_.position.z=short_path_[i](2);
        pose_array.poses.push_back(pos_);
    }
    path_pub_.publish(pose_array);
    return true;
}

void PlannerNode::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // clear existing graph_
    // push back start and end nodes into graph
    // initialize the sampler and use it to get samples
    // create nodes from sampled positions and push back into graph
    // insert all node positions and ids into rtree
    // query rtree for k neighbours of each node
    // insert neighbours and return
    graph_.clear();
    sampler_.init(start, end);

    double p_sample = p_sample_;
    double robot_radius = robot_radius_;
    uint max_samples = p_sample * (start - end).norm() * sampler_.getWidth() / voxel_size_;
    uint num_sample = 0;

    graph_.push_back(Node(new GraphNode(start, 0)));
    graph_.push_back(Node(new GraphNode(end, 1)));

    uint node_id = 2;
    while (num_sample++ < max_samples) {
        Eigen::Vector3d sample = sampler_.generateSample();
        sample(2)= 1;//making height of sample point below the rover's height
        double distance = getMapDistance(sample);
        if ( distance && distance >= robot_radius) {
            graph_.push_back(Node(new GraphNode(sample, node_id++)));
        }
    }

    tree_.clear();
    for (auto& node : graph_) {
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.insert(std::make_pair(pt, node->getID()));
    }

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
}

void PlannerNode::findPath(const uint& start_index, const uint& end_index) {
    // clear existing curr_path_
    // implement A*, keep track of parents in a separate vector
    // once end node is reached, backtrack and push all nodes into path
    if (graph_.empty()) {
        return;
    }
    raw_path_.clear();
    Eigen::Vector3d end_pos = graph_[end_index]->getPosition();
    typedef std::pair<double, uint> f_score_map;

    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(graph_.size(), DBL_MAX);
    std::vector<uint> parent(graph_.size(), INT_MAX);

    open_set.push(std::make_pair((end_pos - graph_[start_index]->getPosition()).norm(), start_index));
    g_score[start_index] = 0.0;
    while (!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = graph_[curr_index]->getPosition();
        open_set.pop();
        if (curr_index -end_index >-15) {
            Path curr_path;
            while (parent[curr_index] != INT_MAX) {
                curr_path.push_back(graph_[curr_index]->getPosition());
                curr_index = parent[curr_index];
            }

            curr_path.push_back(graph_[start_index]->getPosition());
            std::reverse(curr_path.begin(), curr_path.end());
            raw_path_ = curr_path;
            return;
        }

        for (auto& neigh : graph_[curr_index]->getNeighbours()) {
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
}

void PlannerNode::shortenPath() {
    // clear existing short_path_
    // use the collision checker on A* generated path
    if (raw_path_.empty()) {
        return;
    }
    short_path_.clear();
    std::vector<bool> retain(raw_path_.size(), true);
    if(raw_path_.size()>20) findMaximalIndices(0, raw_path_.size() - 1, &retain);//shorten path only if it's size is greater than 20
    ROS_INFO("Working after findMaximal function");
    for (uint i = 0; i < raw_path_.size(); i++) {
        if (retain[i])
            short_path_.push_back(raw_path_[i]);
    }
}

void PlannerNode::findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map) {
    if (start >= end) {
        return;
    }
    if (!isLineInCollision(raw_path_[start], raw_path_[end])) {
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

bool PlannerNode::isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    // check for collisions in the line joining start and end
    // using getMapDistance and robot_radius
    // in increments of voxel size
    //ROS_INFO("isLineInCollision called");
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
}

double PlannerNode::getMapDistance(const Eigen::Vector3d& position){
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
}  // namespace rover::planner
