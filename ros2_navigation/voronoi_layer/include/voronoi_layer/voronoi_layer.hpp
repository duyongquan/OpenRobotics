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

#ifndef VORONOI_LAYER__VORONOI_LAYER_HPP_
#define VORONOI_LAYER__VORONOI_LAYER_HPP_


#include <map>
#include <vector>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

#include "voronoi_layer/dynamic_voronoi.hpp"


namespace nav2_costmap_2d {
class VoronoiLayer : public Layer
{
public:
  VoronoiLayer();
  virtual ~VoronoiLayer();

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;

  void reset() override
  {
    matchSize();
    current_ = false;
  }

  const DynamicVoronoi& getVoronoi() const;
  
  typedef std::recursive_mutex mutex_t;

  mutex_t * getMutex()
  {
    return access_;
  }

private:
  void publishVoronoiGrid(const nav2_costmap_2d::Costmap2D& master_grid);
  void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voronoi_grid_pub_ {nullptr};

  DynamicVoronoi voronoi_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  mutex_t * access_;
};

}  // namespace nav2_costmap_2d

#endif  // VORONOI_LAYER__VORONOI_LAYER_HPP_