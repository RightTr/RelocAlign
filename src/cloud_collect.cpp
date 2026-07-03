#include "ros_utils.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

std::string pcd_path;

void CloudCallback(const ros_utils::PointCloud2MsgConstPtr &cloud_msg)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);  
    pcl::io::savePCDFileBinary(pcd_path, cloud);
    ros_utils::print_info("Saved one frame point cloud with %zu points", cloud.size());
}

int main(int argc, char** argv)
{
    ros_utils::init(argc, argv, "collect_node");

    ros_utils::get_param("pcd_path", pcd_path, std::string{});

    std::string cloud_topic;
    ros_utils::get_param("cloud_topic", cloud_topic, std::string{});
    std::cout << "[Cloud Collect] cloud_topic: " << cloud_topic << std::endl;

    auto sub = ros_utils::subscribe<ros_utils::PointCloud2Msg>(cloud_topic, 1, CloudCallback);
    ros_utils::spin();
    return 0;
}
