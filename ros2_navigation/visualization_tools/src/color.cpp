#include "visualization_tools/color.hpp"

#include <cmath>

namespace visualization_tools 
{

ColorRGBA newColorRGBA(uint8_t red, uint8_t green, uint8_t blue, double alpha) 
{
    std_msgs::msg::ColorRGBA color;
    color.r = red / 255.0;
    color.g = green / 255.0;
    color.b = blue / 255.0;
    color.a = alpha;

    return newColorRGBADouble(red / 255.0, green / 255.0, blue / 255.0, alpha);
}

ColorRGBA newColorRGBADouble(double red, double green, double blue, double alpha) 
{
    std_msgs::msg::ColorRGBA color;
    color.r = red;
    color.g = green;
    color.b = blue;
    color.a = alpha;
    return color;
}

ColorMap::ColorMap(const std::vector<size_t> &color_list) 
{
    if (color_list.size() < 2) {
        throw std::invalid_argument("ColorMap: size of color_list should be at least 2.");
    }

    color_list_ = color_list;
    double value_step = 1.0 / color_list_.size();
    value_list_.clear();

    for (unsigned long i = 0; i < color_list_.size(); i++) {
        value_list_.push_back(value_step * static_cast<double>(i));
    }
    value_list_.back() = 1.0;
}

size_t ColorMap::operator()(double value) 
{
    if (value < 0.0 || value > 1.0) {
    	value = value < 0.0? 0.0:value;
    	value = value > 1.0? 1.0:value;
    	printf("ColorMap: value is out of range [0, 1], modified to %.1f.\n", value);
    }

    unsigned long index = 0;
    for (unsigned long i = 0; i < value_list_.size(); ++i) {
        if (value <= value_list_.at(i) && i != 0) {
            index = i - 1;
        }
    }
    double scale = (value - value_list_.at(index)) / (value_list_.at(index + 1) - value_list_.at(index));

    return colorInterpolation(color_list_.at(index), color_list_.at(index + 1), scale);;
}

size_t ColorMap::colorRGB2Hex(uint8_t red, uint8_t green, uint8_t blue) 
{
    return static_cast<size_t>((red << 16) + (green << 8) + blue);
}

void ColorMap::colorHex2RGB(const size_t &color_hex, uint8_t &red, uint8_t &green, uint8_t &blue) 
{
    red = color_hex >> 16 & 0xFF;
    green = color_hex >> 8 & 0xFF;
    blue = color_hex & 0xFF;
}

size_t ColorMap::colorInterpolation(size_t start, size_t end, double scale)
{
    uint8_t red1, green1, blue1, red2, green2, blue2;
    colorHex2RGB(start, red1, green1, blue1);
    colorHex2RGB(end, red2, green2, blue2);
    auto red_at_value = static_cast<uint8_t>(std::round((1 - scale) * red1 + scale * red2));
    auto green_at_value = static_cast<uint8_t>(std::round((1 - scale) * green1 + scale * green2));
    auto blue_at_value = static_cast<uint8_t>(std::round((1 - scale) * blue1 + scale * blue2));
    return colorRGB2Hex(red_at_value, green_at_value, blue_at_value);
}

void ColorMap::linspaceColorRGBinHex(size_t start, size_t end, size_t num, std::vector<size_t> &color_list) 
{
    if(!color_list.empty()){
        color_list.clear();
    }
    
    double scale;
    double scale_step = 1.0 / (num - 1);
    uint8_t red1, green1, blue1, red2, green2, blue2;
    colorHex2RGB(start, red1, green1, blue1);
    colorHex2RGB(end, red2, green2, blue2);
    for (size_t i = 0; i < num; ++i) {
        scale = i * scale_step;
        auto red_at_value = static_cast<uint8_t>(std::round((1 - scale) * red1 + scale * red2));
        auto green_at_value = static_cast<uint8_t>(std::round((1 - scale) * green1 + scale * green2));
        auto blue_at_value = static_cast<uint8_t>(std::round((1 - scale) * blue1 + scale * blue2));
        color_list.push_back(colorRGB2Hex(red_at_value, green_at_value, blue_at_value));
    }
}

} // namespace ros_viz_tools