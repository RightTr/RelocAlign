#include <ros/ros.h>
#include "relocalign.hpp"
#include "read_configs.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "align_reloc_pub");
    ros::NodeHandle nh;

    std::string config_path;
    ros::param::get("~config_path", config_path);

    RelocAlignConfig relocalignconfig(config_path);

    return 0;
}
