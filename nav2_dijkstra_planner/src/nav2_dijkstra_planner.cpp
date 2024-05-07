
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include "nav2_dijkstra_planner/nav2_dijkstra_planner.hpp"

namespace nav2_dijkstra_planner {

void DijkstraGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  // make pointer to the values in the underlying occupacy grid
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Put parameter initialization here (in case parameters are needed)

  // get the x and y values of the origin of the costmap as double
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();

  // get the x and y sizes of the costmap in cells as unsigned int
  width_ = costmap_->getSizeInCellsX();
  height_ = costmap_->getSizeInCellsY();

  // get the resolution of the costmap as double
  resolution_ = costmap_->getResolution();
  // save the total size of the costmap
  map_size_ = width_ * height_;

  RCLCPP_INFO(node_->get_logger(), "Planner plugin initialized successfully");
  initialized_ = true;
}

void DijkstraGlobalPlanner::cleanup() {
  RCLCPP_INFO(node_->get_logger(), "Clean up plugin %s", name_.c_str());
}

void DijkstraGlobalPlanner::activate() {
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());
}

void DijkstraGlobalPlanner::deactivate() {
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
}

std::unordered_map<int, double>DijkstraGlobalPlanner::find_neighbors(const int &current_node_index, const std::vector<int> &costmap_flat) {
   
    std::unordered_map<int, double> detected_neighbors;
    // length of diagonal = length of one side by the square root of 2 (1.41421)
    double diagonal_step_cost = resolution_ * 1.41421;
    // value used to reject neighbor nodes due to considered obstacle [1-254]
    int lethal_cost = 1;

    // get index value of node above the current node
    int upper = current_node_index - width_;
    // check if this neighbor node lies inside the map boundaries
    // exclude cells that are not inside grid boundaries (e.g. negative values)
    // include the upper cell as a valid neighbor when its index equals to 0
    if (upper >= 0) {
        // check if this neighbor node is an obstacle
        if (costmap_flat[upper] < lethal_cost) {
            // get step cost of moving to this neighbor node
            double step_cost = resolution_ + costmap_flat[upper] / 255.0;
            // add it to the neighbors to be returned by the function
            detected_neighbors.insert({ upper, step_cost });
        }
    }

    // get index value of node to the left of current node
    int left = current_node_index - 1;
    // exclude left neighbor cells that are outside the grid map
    // and exclude left neighbor cells that are on the rightmost column
    if (left % width_ > 0 && left % width_ != width_ - 1) {
        if (costmap_flat[left] < lethal_cost) {
            double step_cost = resolution_ + costmap_flat[left] / 255.0;
            detected_neighbors.insert({ left, step_cost });
        }
    }

    // get index value of node to the upper left of current node
    int upper_left = current_node_index - width_ - 1;
    // exclude cells that are outside the grid map
    // exclude 'upper_left' cells that are on the rightmost column
    if (upper_left >= 0 && left % width_ != width_ - 1) {
        if (costmap_flat[upper_left] < lethal_cost) {
            double step_cost = diagonal_step_cost + costmap_flat[upper_left] / 255.0;
            detected_neighbors.insert({ upper_left, step_cost });
        }
    }

    // get index value of node to the upper right of current node
    int upper_right = current_node_index - width_ + 1;
    // exclude 'upper_right' if not inside grid boundaries (negative values)
    // and exclude it in the first costmap column
    if (upper_right > 0 && (upper_right) % width_ != 0) {
        if (costmap_flat[upper_right] < lethal_cost) {
            double step_cost = diagonal_step_cost + costmap_flat[upper_right] / 255.0;
            detected_neighbors.insert({ upper_right, step_cost });
        }
    }

    // get index value of node to the right of current node
    int right = current_node_index + 1;
    // exclude 'right' neighbor if in the first costmap column
    // and exclude it if beyond the max costmap size
    if (right % width_ != 0 && right >= width_ * height_) {
        if (costmap_flat[right] < lethal_cost) {
            double step_cost = resolution_ + costmap_flat[right] / 255.0;
            detected_neighbors.insert({ right, step_cost });
        }
    }

    // get index value of node to the lower_left of current node
    int lower_left = current_node_index + width_ - 1;
    // exclude lower_left neighbor if it exceedes the max costmap size
    // and exclude it if on the rightmost column
    if (lower_left < height_ * width_ && lower_left % width_ != width_ - 1) {
        if (costmap_flat[lower_left] < lethal_cost) {
            double step_cost = diagonal_step_cost + costmap_flat[lower_left] / 255.0;
            detected_neighbors.insert({ lower_left, step_cost });
        }
    }

    // get index value of node below the current node
    int lower = current_node_index + width_;
    // exclude 'lower' neighbors that exceede the max costmap size
    if (lower < height_ * width_) {
        if (costmap_flat[lower] < lethal_cost) {
            double step_cost = resolution_ + costmap_flat[lower] / 255.0;
            detected_neighbors.insert({ lower, step_cost });
        }
    }

    // get index value of node below and to the right of current node
    int lower_right = current_node_index + width_ + 1;
    // exclude 'lower_right' neighbors that exceede the max costmap size
    // and 'lower_right' neighbors in the first costmap column
    if (lower_right < height_ * width_ && lower_right % width_ != 0) {
        if (costmap_flat[lower_right] < lethal_cost) {
            double step_cost = diagonal_step_cost + costmap_flat[lower_right] / 255.0;
            detected_neighbors.insert({ lower_right, step_cost });
        }
    }

    return detected_neighbors;
}

nav_msgs::msg::Path
DijkstraGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                  const geometry_msgs::msg::PoseStamped &goal) {
  // create empty path
  nav_msgs::msg::Path global_path;

  RCLCPP_INFO(
      node_->get_logger(),
      "Got as start position: %.2f, %.2f, and as goal position: %.2f, %.2f",
      start.pose.position.x, start.pose.position.y, goal.pose.position.x,
      goal.pose.position.y);

  // Check if start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Planner will only accept start position in the %s frame, but "
                 "start position was sent in the %s frame.",
                 global_frame_.c_str(), start.header.frame_id.c_str());
    return global_path;
  }
  // Check if goal state is in the global frame
  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(node_->get_logger(),
                "Planner will only except goal position in the %s frame, but a "
                "goal was sent in the %s frame.",
                global_frame_.c_str(), start.header.frame_id.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // create flat costmap (1-D array occupancy map representation)
  std::vector<int> costmap_flat(map_size_);

  for (size_t idx = 0; idx < map_size_; ++idx) {
    int x, y;
    x = idx % width_;
    y = std::floor(idx / width_);
    costmap_flat.at(idx) = static_cast<int>(costmap_->getCost(x, y));
  }

  // start and goal position as x and y coordinate values
  float start_x = start.pose.position.x;
  float start_y = start.pose.position.y;
  float goal_x = goal.pose.position.x;
  float goal_y = goal.pose.position.y;

  // start and goal position as flat array index representation
  size_t start_index = 0;
  size_t goal_index = 0;

  // check if start/goal world coordinates are inside grid map bounds
  if (inGridMapBounds(start_x, start_y) && inGridMapBounds(goal_x, goal_y)) {
    // convert start_x, start_y, goal_x, goal_y from world coordinates/meters
    // into grid map cell coordinates
    fromWorldToGrid(start_x, start_y);
    fromWorldToGrid(goal_x, goal_y);

    // convert grid map cell coordinates into flat array index values
    start_index = gridCellxyToIndex(start_x, start_y);
    goal_index = gridCellxyToIndex(goal_x, goal_y);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "Start or goal position outside of the map's boundaries");
    return global_path;
  }

  // empty data container that will hold the result of the search
  std::vector<int> shortest_path;

  shortest_path.clear();

  // call Dijkstra algorithm
  if (!dijkstraShortestPath(start_index, goal_index, costmap_flat,
                            shortest_path)) {
    RCLCPP_WARN(node_->get_logger(), "Dijkstra: No path found");
    return global_path;
  };

  // if path found is not empty, convert path from indices to pose waypoints
  if (shortest_path.size()) {
    // insert start node element at the beginning of shortest_path vector
    shortest_path.insert(shortest_path.begin(), start_index);

    for (int p : shortest_path) {
      int x, y;
      fromIndexToGridCellxy(p, x, y);
      float x_waypoint = static_cast<float>(x);
      float y_waypoint = static_cast<float>(y);
      fromGridToWorld(x_waypoint, y_waypoint);

      geometry_msgs::msg::PoseStamped pose_obj;

      pose_obj.pose.position.x = x_waypoint;
      pose_obj.pose.position.y = y_waypoint;
      pose_obj.pose.orientation.x = 0;
      pose_obj.pose.orientation.y = 0;
      pose_obj.pose.orientation.z = 0;
      pose_obj.pose.orientation.w = 1;
      pose_obj.header.stamp = node_->now();
      pose_obj.header.frame_id = global_frame_;
      global_path.poses.push_back(pose_obj);
    }

    // insert goal location to the end of the plan vector
    global_path.poses.push_back(goal);
  }

  return global_path;
}

bool DijkstraGlobalPlanner::dijkstraShortestPath(
    const int &start_cell_index, const int &goal_cell_index,
    const std::vector<int> &costmap_flat, std::vector<int> &shortest_path) {

    // boolean flag that keeps status of the search process
    // true on success, false if goal was not reachable
    bool path_found = false;

    // empty data container that will store nodes and associated g_costs
    std::vector<std::pair<int, double>> open_list;

    // empty set to keep track of already visited/closed nodes
    std::unordered_set<int> closed_list;

    // a blank hash table for mapping children nodes to parents
    std::unordered_map<int, int> parents;

    // an empty hash table to keep node indices and their associated g_cost
    std::unordered_map<int, double> g_costs;

    // keeps index value of current grid cell
    int current_node;

    // set the start's node g_cost
    g_costs[start_cell_index] = 0;

    // put start_index along with it's g_cost into open_list
    open_list.push_back(std::make_pair(start_cell_index, 0.0));

    RCLCPP_INFO(node_->get_logger(), "Dijkstra: Done with initialization");

    // Main loop over open list
    while (!open_list.empty()) {
        // Sort open list by distance cost
        std::sort(open_list.begin(), open_list.end(), []
        (const std::pair<int, double>& a, const std::pair<int, double>& b) 
        {return a.second < b.second;});
        // Update curent node
        current_node = open_list[0].first;
        open_list.erase(open_list.begin());
        closed_list.insert(current_node);
        // Check if goal node has been reached
        if (current_node == goal_cell_index) {
            break;
        }
        // Find neighbors for current node
        else {
            std::unordered_map<int, double> neighbors = find_neighbors(current_node, costmap_flat);
            for (const auto& pair : neighbors) {
                // Skip neighbor if it belongs to the closed list
                if (closed_list.count(pair.first) == 0) {
                    break;
                }
                double neighbor_current_g = g_costs[pair.first];
                // Neighbor exists and has a higher cost
                if (neighbor_current_g != 0 && pair.second < neighbor_current_g) {
                    g_costs[pair.first] = pair.second;
                    parents[pair.first] = current_node;    
                }
                // Neighbor does not exists already
                else if (neighbor_current_g == 0) {
                    g_costs[pair.first] = pair.second;
                    parents[pair.first] = current_node;
                    open_list.push_back(std::make_pair(pair.first, pair.second));
                }
            }
        }
    }

    return true;
}

void DijkstraGlobalPlanner::fromWorldToGrid(float &x, float &y) {
  x = static_cast<size_t>((x - origin_x_) / resolution_);
  y = static_cast<size_t>((y - origin_y_) / resolution_);
}

bool DijkstraGlobalPlanner::inGridMapBounds(const float &x, const float &y) {
  if (x < origin_x_ || y < origin_y_ ||
      x > origin_x_ + (width_ * resolution_) ||
      y > origin_y_ + (height_ * resolution_))
    return false;
  return true;
}

size_t DijkstraGlobalPlanner::gridCellxyToIndex(const float &x,
                                                const float &y) {
  return y * width_ + x;
}

void DijkstraGlobalPlanner::fromIndexToGridCellxy(size_t index, int &x,
                                                  int &y) {
  x = index % width_;
  y = std::floor(index / width_);
}

void DijkstraGlobalPlanner::fromGridToWorld(float &x, float &y) {
  x = x * resolution_ + origin_x_;
  y = y * resolution_ + origin_y_;
}

} // namespace nav2_dijkstra_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dijkstra_planner::DijkstraGlobalPlanner,
                       nav2_core::GlobalPlanner)
