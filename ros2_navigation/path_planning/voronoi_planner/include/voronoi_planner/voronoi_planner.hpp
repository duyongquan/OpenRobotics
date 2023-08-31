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

#ifndef VORONOI_PLANNER__VORONOI_PLANNER_HPP_
#define VORONOI_PLANNER__VORONOI_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "voronoi_layer/voronoi_layer.hpp"
#include "voronoi_planner/nodes.hpp"

namespace voronoi_planner
{
// https://www.matec-conferences.org/articles/matecconf/pdf/2016/05/matecconf_iccma2016_01005.pdf

struct VoronoiData
{
  bool is_voronoi;  // whether the grid is in VD or not
  double dist;      // the distance from the grid to the closest obstacle
};


class VoronoiPlanner : public nav2_core::GlobalPlanner
{
public:
  VoronoiPlanner() = default;
  ~VoronoiPlanner() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /**
   * @brief Voronoi-based planning implementation
   * @param global_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);
  bool plan(VoronoiData** voronoi_diagram, const Node& start, const Node& goal, std::vector<Node>& path);

protected:
  /**
   * @brief search the shortest path from start to VD, or search the shortest path in VD
   * @param start         start node
   * @param goal          goal node
   * @param v_goal        the voronoi node in VD which is closest to start node
   * @param path          shortest path from start to VD
   * @return  true if path found, else false
   */
  bool searchPathWithVoronoi(const Node& start, const Node& goal, std::vector<Node>& path, Node* v_goal = nullptr);

private:
  /**
   * @brief Get permissible motion
   * @return  Node vector of permissible motions
   */
  std::vector<Node> getMotion();

  /**
   * @brief Convert closed list to path
   * @param closed_list closed list
   * @param start       start node
   * @param goal        goal node
   * @return  vector containing path nodes
   */
  std::vector<Node> _convertClosedListToPath(std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list, 
    const Node& start, const Node& goal);

  /**
   * @brief Calculate plan from planning path
   * @param path  path generated by global planner
   * @param plan  plan transfromed from path, i.e. [start, ..., goal]
   * @return  bool true if successful, else false
   */
  bool _getPlanFromPath(std::vector<Node>& path, nav_msgs::msg::Path& plan);
                                              
  /**
   * @brief Transform from grid map(x, y) to grid index(i)
   * @param x grid map x
   * @param y grid map y
   * @return  index
   */
  int grid2Index(int x, int y);

  /**
   * @brief Transform from grid map(x, y) to costmap(x, y)
   * @param gx  grid map x
   * @param gy  grid map y
   * @param mx  costmap x
   * @param my  costmap y
   */
  void map2Grid(double mx, double my, int& gx, int& gy);

   /**
   * @brief Transform a point from world to map frame
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   * @return true if can transform
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  /**
   * @brief Transform a point from map to world frame
   * @param mx double of map X coordinate
   * @param my double of map Y coordinate
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   */
  void mapToWorld(double mx, double my, double & wx, double & wy);

  VoronoiData** voronoi_diagram_;  // voronoi diagram copy
  double circumscribed_radius_;    // the circumscribed radius of robot

  std::shared_ptr<nav2_costmap_2d::VoronoiLayer> voronoi_layer_{nullptr};

  // lethal cost and neutral cost
  unsigned char lethal_cost_, neutral_cost_;

  // pixel number in costmap x, y and total
  int nx_, ny_, ns_;

  // costmap resolution
  double resolution_;

  // obstacle factor(greater means obstacles)
  double factor_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;
};

}  // namespace voronoi_planner

#endif  // VORONOI_PLANNER__VORONOI_PLANNER_HPP_
