#ifndef VISUALIZATION_TOOLS__COLOR_HPP_
#define VISUALIZATION_TOOLS__COLOR_HPP_

#include <std_msgs/msg/color_rgba.hpp>

#include <vector>

namespace visualization_tools 
{

using std_msgs::msg::ColorRGBA;

ColorRGBA newColorRGBA(uint8_t red, uint8_t green, uint8_t blue, double alpha = 1.0);
ColorRGBA newColorRGBADouble(double red, double green, double blue, double alpha = 1.0);

// pre-defined color
const ColorRGBA WHITE      = newColorRGBA(255, 255, 255);
const ColorRGBA BLACK      = newColorRGBA(  0,   0,   0);
const ColorRGBA RED        = newColorRGBA(255,   0,   0);
const ColorRGBA GREEN      = newColorRGBA(  0, 255,   0);
const ColorRGBA BLUE       = newColorRGBA(  0,   0, 255);
const ColorRGBA YELLOW     = newColorRGBA(255, 255,   0);
const ColorRGBA CYAN       = newColorRGBA(  0, 255, 255);
const ColorRGBA MAGENTA    = newColorRGBA(255,   0, 255);
const ColorRGBA GRAY       = newColorRGBA(128, 128, 128);
const ColorRGBA PURPLE     = newColorRGBA(128,   0, 128);
const ColorRGBA PINK       = newColorRGBA(255, 192, 203);
const ColorRGBA LIGHT_BLUE = newColorRGBA(173, 216, 230);
const ColorRGBA LIME_GREEN = newColorRGBA( 50, 205,  50);
const ColorRGBA SLATE_GRAY = newColorRGBA(112, 128, 144);

class ColorMap 
{
public:
    ColorMap(const std::vector<size_t> &color_list);
    size_t operator()(double value);
    
    static size_t colorRGB2Hex(uint8_t red, uint8_t green, uint8_t blue);
    static void colorHex2RGB(const size_t &color_hex, uint8_t &red, uint8_t &green, uint8_t &blue);
    static void linspaceColorRGBinHex(size_t start, size_t end, size_t num, std::vector<size_t> &color_list);

private:
    static size_t colorInterpolation(size_t start, size_t end, double scale);

    std::vector<double> value_list_;
    std::vector<size_t> color_list_;
};

} // namespace visualization_tools

#endif //  VISUALIZATION_TOOLS__COLOR_HPP_