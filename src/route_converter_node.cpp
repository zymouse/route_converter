#include "route_converter/route_converter.hpp"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "route_converter");
    route_converter::route_converter route_converter_node;
    ros::spin();

    return 0;
}