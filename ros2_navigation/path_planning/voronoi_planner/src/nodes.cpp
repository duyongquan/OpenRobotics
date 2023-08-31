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

#include "voronoi_planner/nodes.hpp"

namespace voronoi_planner
{
/**
 * @brief Constructor for Node class
 * @param x   x value
 * @param y   y value
 * @param g   g value, cost to get to this node
 * @param h   h value, heuritic cost of this node
 * @param id  node's id
 * @param pid node's parent's id
 */
Node::Node(int x, int y, double g, double h, int id, int pid) : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid)
{
}

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return  Node with current node's and input node n's values added
 */
Node Node::operator+(const Node& n) const
{
  Node result;
  result.x_ = x_ + n.x_;
  result.y_ = y_ + n.y_;
  result.g_ = g_ + n.g_;

  return result;
}

/**
 * @brief Overloading operator - for Node class
 * @param n another Node
 * @return  Node with current node's and input node n's values subtracted
 */
Node Node::operator-(const Node& n) const
{
  Node result;
  result.x_ = x_ - n.x_;
  result.y_ = y_ - n.y_;

  return result;
}

/**
 * @brief Overloading operator == for Node class
 * @param n another Node
 * @return  true if current node equals node n, else false
 */
bool Node::operator==(const Node& n) const
{
  return x_ == n.x_ && y_ == n.y_;
}

/**
 * @brief Overloading operator != for Node class
 * @param n another Node
 * @return  true if current node equals node n, else false
 */
bool Node::operator!=(const Node& n) const
{
  return !operator==(n);
}

/**
 * @brief Overlaod () operator to calculate the hash of a Node
 * @param n Node for which the hash is to be calculated
 * @return  hash value, node id
 */
size_t NodeIdAsHash::operator()(const Node& n) const
{
  return n.id_;
}

/**
 * @brief Compare cost between 2 nodes
 * @param n1  one Node
 * @param n2  another Node
 * @return  true if the cost to get to n1 is greater than n2, else false
 */
bool compare_cost::operator()(const Node& n1, const Node& n2) const
{
  // Can modify this to allow tie breaks based on heuristic cost if required
  return (n1.g_ + n1.h_ > n2.g_ + n2.h_) || ((n1.g_ + n1.h_ == n2.g_ + n2.h_) && (n1.h_ > n2.h_));
}

/**
 * @brief Compare coordinates between 2 nodes
 * @param n1  one Node
 * @param n2  another Node
 * @return  true if n1 equals n2, else false
 */
bool compare_coordinates::operator()(const Node& n1, const Node& n2) const
{
  return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
}

}  // namespace voronoi_planner