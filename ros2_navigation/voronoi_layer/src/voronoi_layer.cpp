/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "voronoi_layer/voronoi_layer.hpp"

#include <chrono>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d {

VoronoiLayer::VoronoiLayer()
{
}

VoronoiLayer::~VoronoiLayer()
{
}


void VoronoiLayer::onInitialize()
{
  current_ = true;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  voronoi_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "voronoi_grid", rclcpp::SystemDefaultsQoS());
}

void VoronoiLayer::matchSize()
{

}

const DynamicVoronoi& VoronoiLayer::getVoronoi() const
{
  return voronoi_;
}

void VoronoiLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;
  (void)min_x;
  (void)min_y;
  (void)max_x;
  (void)max_y;

  if (!enabled_)
  {
    return;
  }
}


void VoronoiLayer::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
  unsigned char* pc = costarr;
  for (int i = 0; i < nx; i++)
  {
    *pc++ = value;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++)
  {
    *pc++ = value;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx)
  {
    *pc = value;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx)
  {
    *pc = value;
  }
}

void VoronoiLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid, 
  int min_i, int min_j, int max_i, int max_j)
{
  (void)min_i;
  (void)min_j;
  (void)max_i;
  (void)max_j;

  if (!enabled_)
  {
    return;
  }

  // std::unique_lock<std::mutex> lock(mutex_);

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  outlineMap(master_grid.getCharMap(), size_x, size_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  if (last_size_x_ != size_x || last_size_y_ != size_y)
  {
    voronoi_.initializeEmpty(size_x, size_y);

    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  std::vector<IntPoint> new_free_cells, new_occupied_cells;
  for (unsigned int j = 0; j < size_y; ++j)
  {
    for (unsigned int i = 0; i < size_x; ++i)
    {
      if (voronoi_.isOccupied(i, j) && master_grid.getCost(i, j) == nav2_costmap_2d::FREE_SPACE)
      {
        new_free_cells.push_back(IntPoint(i, j));
      }

      if (!voronoi_.isOccupied(i, j) && master_grid.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        new_occupied_cells.push_back(IntPoint(i, j));
      }
    }
  }

  for (size_t i = 0; i < new_free_cells.size(); ++i)
  {
    voronoi_.clearCell(new_free_cells[i].x, new_free_cells[i].y);
  }

  for (size_t i = 0; i < new_occupied_cells.size(); ++i)
  {
    voronoi_.occupyCell(new_occupied_cells[i].x, new_occupied_cells[i].y);
  }

  voronoi_.update();
  voronoi_.prune();

  publishVoronoiGrid(master_grid);
}

void VoronoiLayer::publishVoronoiGrid(const nav2_costmap_2d::Costmap2D& master_grid)
{
  unsigned int nx = master_grid.getSizeInCellsX();
  unsigned int ny = master_grid.getSizeInCellsY();

  double resolution = master_grid.getResolution();
  nav_msgs::msg::OccupancyGrid grid;
  // Publish Whole Grid
  grid.header.frame_id = "map";
  grid.header.stamp = rclcpp::Clock().now();
  grid.info.resolution = resolution;

  grid.info.width = nx;
  grid.info.height = ny;

  grid.info.origin.position.x = master_grid.getOriginX();
  grid.info.origin.position.y = master_grid.getOriginY();
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(nx * ny);

  for (unsigned int x = 0; x < nx; x++)
  {
    for (unsigned int y = 0; y < ny; y++)
    {
      if (voronoi_.isVoronoi(x, y))
      {
        grid.data[x + y * nx] = 127;
      }
      else
      {
        grid.data[x + y * nx] = 0;
      }
    }
  }
  voronoi_grid_pub_->publish(grid);
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VoronoiLayer, nav2_costmap_2d::Layer)
