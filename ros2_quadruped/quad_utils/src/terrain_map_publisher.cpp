#include "quad_utils/terrain_map_publisher.hpp"

namespace ros2_quadruped {
namespace quad_utils {

TerrainMapPublisher::TerrainMapPublisher() : Node("terrain_map_publisher")
{
    map_converter_ = std::make_shared<MeshToGridMapConverter>(this);
}

TerrainMapPublisher::~TerrainMapPublisher() 
{
}

}  // namespace quad_utils
}  // namespace ros2_quadruped