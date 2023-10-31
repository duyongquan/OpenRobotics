
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace bspline
{
namespace
{

class NonUniformBsplineNode : public rclcpp::Node
{
public:
    NonUniformBsplineNode() : Node("non_uniform_bspline")
    {

    }

private:

};

}  // namespace 
}  // namespace bspline


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<bspline::NonUniformBsplineNode>();


    // Executor add node 
    executor.add_node(node);

    // Spin here
    executor.spin();
    rclcpp::shutdown();
    return 0;
}