#include "nav2_demos/tutorials_nav2_utils_dataset_loader.hpp"
#include "nav2_demos/config.h"

#include <exception>
#include <iostream>

namespace ros2_tutorials
{
namespace nav2
{

std::string GetImagePath()
{
    return kSourceDirectory + "/images/";
}

std::string GetMapsPath()
{
    return kSourceDirectory + "/maps/";
}

cv::Mat LoadImage(const std::string& filename, bool show)
{
    cv::Mat image = cv::imread(GetImagePath() + filename, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::string error = "Load image : " +  filename + " error";
        throw error;
    }

    std::cout << "opencv image " << filename << " [width, height] : " 
              << "["  << image.cols
              << ", " << image.rows
              << "]" << std::endl;

    if (show) {
        cv::imshow("image", image);
        cv::waitKey(3);
    }
    return image;
}

}  // namespace nav2
}  // namespace ros2_tutorials
