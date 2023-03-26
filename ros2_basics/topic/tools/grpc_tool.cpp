#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp/rclcpp.hpp"

#include "grpc_tool.hpp"

static constexpr char OPTION_HELP[] = "-h";
static constexpr char OPTION_TEST_STAGE_STATIC_ONE[] = "-o";
static constexpr char OPTION_TEST_STAGE_STATIC_TWO[] = "-t";
static constexpr char OPTION_TEST_STAGE_MOTION[] = "-m";
static constexpr char OPTION_SET_NETWORK[] = "-n";
static constexpr char OPTION_SET_WIFI[] = "-w";

namespace cyberdog
{

GrpcTool::GrpcTool()
{

}

GrpcTool::~GrpcTool()
{

}

void GrpcTool::PrintUsage(const char * progname)
{
    std::cout << progname << " [OPTIONS]" << std::endl <<
        std::endl << "Options when starting the GRPC server tool:" << std::endl <<
        std::left << std::setw(14) << std::setfill(' ') << OPTION_HELP <<
        "print this help message" << std::endl <<
        std::left << std::setw(14) << std::setfill(' ') << OPTION_TEST_STAGE_STATIC_ONE <<
            "test stage static one" << std::endl <<
        std::left << std::setw(14) << std::setfill(' ') << OPTION_TEST_STAGE_STATIC_TWO <<
            "test stage static two" << std::endl <<
        std::left << std::setw(14) << std::setfill(' ') << OPTION_TEST_STAGE_MOTION <<
            "test stage motion" << std::endl <<
        std::left << std::setw(14) << std::setfill(' ') << OPTION_SET_NETWORK <<
            "set network config" << std::endl <<
        std::left << std::setw(14) << std::setfill(' ') << OPTION_SET_WIFI <<
            "set network wifi" << std::endl <<
        std::endl << std::endl;
}


void GrpcTool::StartTestMotion()
{
    std::cout << "[Motion] " << std::endl;
}

void GrpcTool::StartTestStaticOne()
{
    std::cout << "[StaticOne] " << std::endl;
}

void GrpcTool::StartTestStaticTwo()
{
    std::cout << "[StaticTwo] " << std::endl;
}

void GrpcTool::SetIP(const std::string & ip_addr)
{
    std::cout << "[IP] " << ip_addr << std::endl;
}

void GrpcTool::SetWifiName(const std::string & wifi_name)
{
    std::cout << "[WIFI] " << wifi_name << std::endl;
}

}  // namespace cyberdog


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