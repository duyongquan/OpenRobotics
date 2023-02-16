#ifndef ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_DATASET_LOADER_HPP_
#define ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_DATASET_LOADER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "cv_bridge/cv_bridge.h"

namespace ros2_tutorials
{
namespace nav2
{

std::string GetImagePath();

cv::Mat LoadImage(const std::string& filename, bool show = false);

}  // namespace nav2
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_DATASET_LOADER_HPP_