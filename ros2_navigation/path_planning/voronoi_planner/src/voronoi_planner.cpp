/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "voronoi_planner/voronoi_planner.hpp"

namespace voronoi_planner {

void VoronoiPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nx_ = costmap_->getSizeInCellsX();
  ny_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  circumscribed_radius_ = costmap_ros->getLayeredCostmap()->getCircumscribedRadius();

  voronoi_layer_ = std::make_shared<nav2_costmap_2d::VoronoiLayer>();
}

void VoronoiPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void VoronoiPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void VoronoiPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path VoronoiPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // clear the plan, just in case
  global_path.poses.clear();

  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // tranform from costmap to grid map
  int g_start_x, g_start_y;
  int g_goal_x,  g_goal_y;

  map2Grid(start.pose.position.x, start.pose.position.y, g_start_x, g_start_y);
  map2Grid(goal.pose.position.x, goal.pose.position.y, g_goal_x, g_goal_y);

  // NOTE: how to init start and goal?
  voronoi_planner::Node start_node(g_start_x, g_start_y, 0, 0, grid2Index(g_start_x, g_start_y), 0);
  voronoi_planner::Node goal_node(g_goal_x, g_goal_y, 0, 0, grid2Index(g_goal_x, g_goal_y), 0);

  // calculate path
  std::vector<voronoi_planner::Node> path;
  std::vector<voronoi_planner::Node> expand;
  bool path_found = false;


  VoronoiData** voronoi_diagram;
  voronoi_diagram = new VoronoiData*[nx_];
  for (unsigned int i = 0; i < nx_; i++)
    voronoi_diagram[i] = new VoronoiData[ny_];

  // std::unique_lock<std::mutex> lock(voronoi_layer_->getMutex());
  const DynamicVoronoi& voronoi = voronoi_layer_->getVoronoi();
  for (unsigned int j = 0; j < ny_; j++)
  {
    for (unsigned int i = 0; i < nx_; i++)
    {
      voronoi_diagram[i][j].dist = voronoi.getDistance(i, j) * resolution_;
      voronoi_diagram[i][j].is_voronoi = voronoi.isVoronoi(i, j);
    }
  }

  path_found = plan(voronoi_diagram, start_node, goal_node, path);
  if (path_found)
  {
    if (_getPlanFromPath(path, global_path))
    {
      geometry_msgs::msg::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = node_->now();
      global_path.poses.push_back(goalCopy);
    }
    else {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to get a plan from path when a legal path was found. This shouldn't happen.");
    }
  }

  return global_path;
}

/**
 * @brief Voronoi-based planning implementation
 * @param global_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool VoronoiPlanner::plan(const unsigned char* global_costmap, const Node& start, const Node& goal,
                          std::vector<Node>& path, std::vector<Node>& expand)
{
  return true;
}

bool VoronoiPlanner::plan(VoronoiData** voronoi_diagram, const Node& start, const Node& goal, std::vector<Node>& path)
{
  voronoi_diagram_ = voronoi_diagram;

  // clear vector
  path.clear();

  // start/goal to Voronoi Diagram, shortest path in Voronoi Diagram
  std::vector<Node> path_s, path_g, path_v;

  // start/goal point in Voronoi Diagram
  Node v_start, v_goal;

  if (!searchPathWithVoronoi(start, goal, path_s, &v_start))
    return false;

  if (!searchPathWithVoronoi(goal, start, path_g, &v_goal))
    return false;
  std::reverse(path_g.begin(), path_g.end());

  if (!searchPathWithVoronoi(v_start, v_goal, path_v))
    return false;

  path_g.insert(path_g.end(), path_v.begin(), path_v.end());
  path_g.insert(path_g.end(), path_s.begin(), path_s.end());
  path = path_g;

  return true;
}

/**
 * @brief search the shortest path from start to VD, or search the shortest path in VD
 * @param start         start node
 * @param goal          goal node
 * @param v_goal        the voronoi node in VD which is closest to start node
 * @param path          shortest path from start to VD
 * @return  true if path found, else false
 */
bool VoronoiPlanner::searchPathWithVoronoi(const Node& start, const Node& goal, std::vector<Node>& path, Node* v_goal)
{
  path.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

  open_list.push(start);

  // get all possible motions
  const std::vector<Node> motion = getMotion();

  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current) != closed_list.end())
      continue;

    closed_list.insert(current);

    // goal found
    if ((current == goal) || (v_goal == nullptr ? false : voronoi_diagram_[current.x_][current.y_].is_voronoi))
    {
      path = _convertClosedListToPath(closed_list, start, current);
      if (v_goal != nullptr)
      {
        v_goal->x_ = current.x_;
        v_goal->y_ = current.y_;
        v_goal->id_ = current.id_;
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motion)
    {
      Node node_new = current + m;

      // current node do not exist in closed list
      if (closed_list.find(node_new) != closed_list.end())
        continue;

      // explore a new node
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);
      node_new.pid_ = current.id_;

      // next node hit the boundary or obstacle
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_) ||
          (voronoi_diagram_[node_new.x_][node_new.y_].dist < circumscribed_radius_))
        continue;

      // search in VD
      if ((v_goal == nullptr) && (!voronoi_diagram_[node_new.x_][node_new.y_].is_voronoi))
        continue;

      node_new.h_ = std::hypot(node_new.x_ - goal.x_, node_new.y_ - goal.y_);

      open_list.push(node_new);
    }
  }
  return false;
}

/**
 * @brief Get permissible motion
 * @return  Node vector of permissible motions
 */
std::vector<Node> VoronoiPlanner::getMotion()
{
  return { Node(0, 1, 1),
           Node(1, 0, 1),
           Node(0, -1, 1),
           Node(-1, 0, 1),
           Node(1, 1, std::sqrt(2)),
           Node(1, -1, std::sqrt(2)),
           Node(-1, 1, std::sqrt(2)),
           Node(-1, -1, std::sqrt(2)) };
}

/**
 * @brief Convert closed list to path
 * @param closed_list closed list
 * @param start       start node
 * @param goal        goal node
 * @return  vector containing path nodes
 */
std::vector<Node> VoronoiPlanner::_convertClosedListToPath(
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list, const Node& start, const Node& goal)
{
  auto current = *closed_list.find(goal);
  
  std::vector<Node> path;
  while (current != start)
  {
    path.push_back(current);
    auto it = closed_list.find(Node(current.pid_ % nx_, current.pid_ / ny_, 0, 0, current.pid_));
    if (it != closed_list.end())
      current = *it;
    else
      return {};
  }
  path.push_back(start);

  return path;
}

bool VoronoiPlanner::_getPlanFromPath(std::vector<Node>& path, nav_msgs::msg::Path& plan)
{
  plan.poses.clear();

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    mapToWorld((double)path[i].x_, (double)path[i].y_, wx, wy);

    // coding as message type
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

int VoronoiPlanner::grid2Index(int x, int y)
{
  return x + nx_ * y;
}

void VoronoiPlanner::map2Grid(double mx, double my, int& gx, int& gy)
{
  gx = (int)mx;
  gy = (int)my;
}

bool VoronoiPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void VoronoiPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

}  // namespace voronoi_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlanner, nav2_core::GlobalPlanner)
