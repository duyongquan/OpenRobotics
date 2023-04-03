#include "grpc_tool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp/rclcpp.hpp"

static constexpr char OPTION_HELP[] = "-h";
static constexpr char OPTION_TEST_STAGE_STATIC_ONE[] = "-o";
static constexpr char OPTION_TEST_STAGE_STATIC_TWO[] = "-t";
static constexpr char OPTION_TEST_STAGE_MOTION[] = "-m";
static constexpr char OPTION_SET_NETWORK[] = "-n";
static constexpr char OPTION_SET_WIFI[] = "-w";

int main(int argc, char * argv[])
{
    auto tool = std::make_shared<cyberdog::GrpcTool>();

    // Optional argument parsing
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_HELP)) {
        tool->PrintUsage(argv[0]);
        return 0;
    }
    
    // set network
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_SET_NETWORK)) {
        auto ip_addr = rcutils_cli_get_option(argv, argv + argc, OPTION_SET_NETWORK);
        tool->SetIP(ip_addr);
    }

    // set wifi
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_SET_WIFI)) {
        auto wifi = rcutils_cli_get_option(argv, argv + argc, OPTION_SET_WIFI);
        tool->SetWifiName(wifi);
    }

    // test stage static one
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TEST_STAGE_STATIC_ONE)) {
        auto data = rcutils_cli_get_option(argv, argv + argc, OPTION_TEST_STAGE_STATIC_ONE);
        tool->StartTestStaticOne();
    }

    // test stage static two
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TEST_STAGE_STATIC_TWO)) {
        auto data = rcutils_cli_get_option(argv, argv + argc, OPTION_TEST_STAGE_STATIC_TWO);
        tool->StartTestStaticTwo();
    }

    // test stage motion
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TEST_STAGE_MOTION)) {
        auto data = rcutils_cli_get_option(argv, argv + argc, OPTION_TEST_STAGE_MOTION);
        tool->StartTestMotion();
    }

    return 0;
}