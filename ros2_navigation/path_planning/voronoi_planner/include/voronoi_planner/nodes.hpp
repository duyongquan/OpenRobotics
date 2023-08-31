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

#ifndef VORONOI_PLANNER__NODES_HPP_
#define VORONOI_PLANNER__NODES_HPP_

#define INF 10000  // infinity, a big enough number

#include <cmath>
#include <array>
#include <vector>
#include <map>
#include <unordered_set>

namespace voronoi_planner
{
/**
 * @brief Basic Node class
 */
class Node
{
public:
  /**
   * @brief Constructor for Node class
   * @param x   x value
   * @param y   y value
   * @param g   g value, cost to get to this node
   * @param h   h value, heuritic cost of this node
   * @param id  node's id
   * @param pid node's parent's id
   */
  Node(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0);

  /**
   * @brief Overloading operator + for Node class
   * @param n another Node
   * @return  Node with current node's and input node n's values added
   */
  Node operator+(const Node& n) const;

  /**
   * @brief Overloading operator - for Node class
   * @param n another Node
   * @return  Node with current node's and input node n's values subtracted
   */
  Node operator-(const Node& n) const;

  /**
   * @brief Overloading operator == for Node class
   * @param n another Node
   * @return  true if current node equals node n, else false
   */
  bool operator==(const Node& n) const;

  /**
   * @brief Overloading operator != for Node class
   * @param n another Node
   * @return  true if current node equals node n, else false
   */
  bool operator!=(const Node& n) const;

public:
  int x_, y_;     // x and y value
  double g_, h_;  // g value, cost to reach this node. h value, heuristic cost to reach the goal
  int id_, pid_;  // Node's index and parent's index
};

/**
 * @brief Hash for node struct that returns node id
 */
class NodeIdAsHash
{
public:
  /**
   * @brief Overlaod () operator to calculate the hash of a Node
   * @param n Node for which the hash is to be calculated
   * @return  hash value, node id
   */
  size_t operator()(const Node& n) const;
};

/**
 * @brief Struct created to encapsulate function compare cost between 2 Nodes.
 *        Used in with multiple algorithms and classes
 */
struct compare_cost
{
  /**
   * @brief Compare cost between 2 nodes
   * @param n1  one Node
   * @param n2  another Node
   * @return  true if the cost to get to n1 is greater than n2, else false
   */
  bool operator()(const Node& n1, const Node& n2) const;
};

/**
 * @brief Struct created to encapsulate function compare coordinates between 2 Nodes.
 *        Used in with multiple algorithms and classes
 */
struct compare_coordinates
{
  /**
   * @brief Compare coordinates between 2 nodes
   * @param n1  one Node
   * @param n2  another Node
   * @return  true if n1 equals n2, else false
   */
  bool operator()(const Node& n1, const Node& n2) const;
};

/**
 * @brief Structure to generate a hash for std::pair
 * @details This allows the use of pairs in data structures that use a hash,
 * such as unordered_map/set
 */
struct pair_hash
{
  /**
   * @brief Function used to generate hash for keys
   * @param pair pair of values
   * @return generated hash value
   */
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const
  {
    return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
  }
};

/* =====================================================================================
 * Some other nodes defined by user.
   =====================================================================================*/

/**
 * @brief 2-d Node in plane, used in kd-tree search
 */
class PlaneNode : public Node, public std::array<int, 2>
{
public:
  /**
   * @brief Construct a new Plane Node object
   * @param x   x value
   * @param y   y value
   * @param g   g value, cost to get to this node
   * @param h   h value, heuritic cost of this node
   * @param id  node's id
   * @param pid node's parent's id
   */
  PlaneNode(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0) : Node(x, y, g, h, id, pid)
  {
    (*this)[0] = x;
    (*this)[1] = y;
  }

  /**
   * @brief Construct a new Plane Node object
   * @param n another Node
   */
  PlaneNode(const Node& n) : PlaneNode(n.x_, n.y_, n.g_, n.h_, n.id_, n.pid_)
  {
  }

  static const int dim = 2;
};

class DNode : public Node
{
public:
  enum Tag
  {
    NEW = 0,
    OPEN = 1,
    CLOSED = 2
  };
  /**
   * @brief Construct a new DNode object
   * @param x       X value
   * @param y       Y value
   * @param cost    Cost to get to this node
   * @param h_cost  Heuritic cost of this node
   * @param id      Node's id
   * @param pid     Node's parent's id
   * @param t       Node's tag among enum Tag
   * @param k       Node's k_min in history
   */
  DNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const int t = NEW, const double k = INF)
    : Node(x, y, cost, h_cost, id, pid), t_(t), k_(k)
  {
  }

public:
  int t_;     // Node's tag among enum Tag
  double k_;  // Node's k_min in history
};

class LNode : public Node
{
public:
  /**
   * @brief Construct a new LNode object
   *
   * @param x       X value
   * @param y       Y value
   * @param cost    Cost to get to this node
   * @param h_cost  Heuritic cost of this node
   * @param id      Node's id
   * @param pid     Node's parent's id
   * @param rhs     Node's right hand side
   * @param key     Node's key value
   */
  LNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const double rhs = INF, const double key = INF)
    : Node(x, y, cost, h_cost, id, pid), rhs(rhs), key(key)
  {
  }

public:
  double rhs;                                         // minimum cost moving from start(value)
  double key;                                         // priority
  std::multimap<double, LNode*>::iterator open_it;  // iterator
};

}  // namespace voronoi_planner
#endif  // VORONOI_PLANNER__NODES_HPP_