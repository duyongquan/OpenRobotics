#include "visualization_tools/ros_visualization_tools.hpp"
#include <random>

static std::default_random_engine e;
std::uniform_int_distribution<int> randRGB(0, 255);

using visualization_tools::RosVizTools;
using visualization_tools::ColorMap;

namespace visualization_tools
{
namespace
{

class NodeTest : public rclcpp::Node
{
public:
    NodeTest() : Node("node_test")
    {
        Run();
    }

    void Run()
    {
        std::string topic = "demo_marker";
        visualization_tools::RosVizTools markers(this, topic);
        std::string frame_id = "ros_viz_tools";
        std::string ns;

        rclcpp::WallRate r(1);
        while (rclcpp::ok())
        {
            markers.clear();

            // Frame (Axes)
            visualization_msgs::msg::Marker marker_frame1, marker_frame2;
            ns = "axes";
            geometry_msgs::msg::Pose pose;
            pose.position.x = -3.0;
            pose.position.y = 1.0;
            pose.position.z = -1.0;

            tf2::Quaternion quaternion;
            quaternion.setRPY(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();

            marker_frame1 = RosVizTools::newFrame(0.1, 2.0, pose, ns, 0, frame_id);
            pose.position.x = -5.0;
            pose.position.y = 2.0;
            pose.position.z = -3.0;
            quaternion.setRPY(30 * M_PI / 180, 30 * M_PI / 180, 30 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();

            marker_frame2 = RosVizTools::newFrame(0.1, 1.0, pose, ns, 1, frame_id);
            markers.append(marker_frame1);
            markers.append(marker_frame2);

            // Cube List
            ns = "cube_list";
            visualization_msgs::msg::Marker marker_cubelist = RosVizTools::newCubeList(0.5, ns, 0, visualization_tools::WHITE, frame_id);
            for (int i = 0; i < 10; ++i) {
                geometry_msgs::msg::Point p;
                p.x = i;
                p.y = pow(p.x, 2.0);
                p.z = 1.0;
                marker_cubelist.points.push_back(p);
                std_msgs::msg::ColorRGBA color = visualization_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
                marker_cubelist.colors.push_back(color);
            }
            markers.append(marker_cubelist);

            // Line Strip
            ns = "line_strip";
            visualization_msgs::msg::Marker marker_linestrip = RosVizTools::newLineStrip(0.3, ns, 0, visualization_tools::LIGHT_BLUE, frame_id);
            for (int i = 0; i < 10; ++i) {
                geometry_msgs::msg::Point p;
                p.x = i;
                p.y = pow(p.x, 2.0);
                p.z = 2.0;
                marker_linestrip.points.push_back(p);
                std_msgs::msg::ColorRGBA color = visualization_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
                marker_linestrip.colors.push_back(color);
            }
            markers.append(marker_linestrip);

            // Sphere List
            ns = "sphere_list";
            visualization_msgs::msg::Marker marker_spherelist = RosVizTools::newSphereList(.0, ns, 0, visualization_tools::LIME_GREEN, frame_id);
            for (int i = 0; i < 10; ++i) {
                geometry_msgs::msg::Point p;
                p.x = i;
                p.y = pow(p.x, 2.0);
                p.z = 3.5;
                marker_spherelist.points.push_back(p);
                std_msgs::msg::ColorRGBA color = visualization_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
                marker_spherelist.colors.push_back(color);
            }
            markers.append(marker_spherelist);

            // Text
            ns = "text";
            pose.position.x = -2.0;
            pose.position.y = 2.0;
            pose.position.z = 2.0;
            quaternion.setRPY(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();

            visualization_msgs::msg::Marker marker_text = RosVizTools::newText(1.0, pose, ns, 0, visualization_tools::WHITE, frame_id);
            marker_text.text = "This is text marker.";
            markers.append(marker_text);

            // Cylinder
            ns = "cylinder";
            geometry_msgs::msg::Vector3 scale;
            scale.x = 0.5;
            scale.y = 0.5;
            scale.z = 1.0;
            pose.position.x = -2.0;
            pose.position.y = -2.0;
            pose.position.z = -2.0;
            quaternion.setRPY(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();

            visualization_msgs::msg::Marker marker_cylinder = RosVizTools::newCylinder(scale, pose , ns, 0, visualization_tools::WHITE, frame_id);
            markers.append(marker_cylinder);

            // Cube
            ns = "cube";
            pose.position.x = -1.0;
            pose.position.y = -1.0;
            pose.position.z = -1.0;
            quaternion.setRPY(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();
            visualization_msgs::msg::Marker marker_cube = RosVizTools::newCube(1.0, pose , ns, 0, visualization_tools::WHITE, frame_id);
            markers.append(marker_cube);

            // Cube
            ns = "sphere";
            pose.position.x = -3.0;
            pose.position.y = -3.0;
            pose.position.z = -3.0;
            quaternion.setRPY(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();
            visualization_msgs::msg::Marker marker_sphere = RosVizTools::newSphere(0.5, pose , ns, 0, visualization_tools::RED, frame_id);
            markers.append(marker_sphere);

            // Arrow
            ns = "arrow";
            scale.x = 1.0;
            scale.y = 0.1;
            scale.z = 0.1;
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            quaternion.setRPY(0 * M_PI / 180, 0 * M_PI / 180, 90 * M_PI / 180);
            pose.orientation.x = quaternion.getX();
            pose.orientation.y = quaternion.getY();
            pose.orientation.z = quaternion.getZ();
            pose.orientation.w = quaternion.getW();
            visualization_msgs::msg::Marker marker_arrow = RosVizTools::newArrow(scale, pose , ns, 0, visualization_tools::WHITE, frame_id);
            markers.append(marker_arrow);

            // ColorMap
            size_t start_color = ColorMap::colorRGB2Hex(255, 0, 0);
            size_t end_color = ColorMap::colorRGB2Hex(0, 255, 0);
            std::vector<size_t> color_list1, color_list2;
            ColorMap::linspaceColorRGBinHex(start_color, end_color, 2, color_list1);
            ColorMap colormap1(color_list1);
            color_list2.push_back(ColorMap::colorRGB2Hex(255, 0, 0));
            color_list2.push_back(ColorMap::colorRGB2Hex(0, 255, 0));
            ColorMap colormap2(color_list2);
            ns = "colormap";
            visualization_msgs::msg::Marker marker_colormap1 = RosVizTools::newLineStrip(0.2, ns, 0, visualization_tools::LIGHT_BLUE, frame_id);
            visualization_msgs::msg::Marker marker_colormap2 = RosVizTools::newSphereList(0.2, ns, 1, visualization_tools::LIGHT_BLUE, frame_id);
            for (int i = 0; i < 20; ++i) {
                geometry_msgs::msg::Point p;
                p.x = i * 0.5;
                p.y = sin(p.x);
                p.z = -1.0;
                auto scale_value = (p.y + 1.0) / 2.0;
                marker_colormap1.points.push_back(p);
                uint8_t red, green, blue;
                ColorMap::colorHex2RGB(colormap1(scale_value), red, green, blue);
                std_msgs::msg::ColorRGBA color = visualization_tools::newColorRGBA(red, green, blue);
                marker_colormap1.colors.push_back(color);
                p.z = -2.0;
                marker_colormap2.points.push_back(p);
                ColorMap::colorHex2RGB(colormap2(scale_value), red, green, blue);
                color = visualization_tools::newColorRGBA(red, green, blue);
                marker_colormap2.colors.push_back(color);

            }
            markers.append(marker_colormap1);
            markers.append(marker_colormap2);

            // publish
            markers.publish();

            r.sleep();
        }
    }
};

}  // namespace 
}  // namespace visualization_tools



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<visualization_tools::NodeTest>();


    // Executor add node 
    executor.add_node(node);

    // Spin here
    executor.spin();
    rclcpp::shutdown();
    return 0;
}