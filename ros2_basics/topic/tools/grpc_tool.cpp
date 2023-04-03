#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp/rclcpp.hpp"


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
