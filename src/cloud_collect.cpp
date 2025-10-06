#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

std::string pcd_path;

void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);  
    pcl::io::savePCDFileBinary(pcd_path, cloud);
    ROS_INFO("Saved one frame point cloud with %zu points", cloud.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collect_node");
    ros::NodeHandle nh;

    ros::param::get("pcd_path", pcd_path);

    std::string cloud_topic;
    ros::param::get("cloud_topic", cloud_topic);
    std::cout << "[Cloud Collect] cloud_topic: " << cloud_topic << std::endl;

    bool is_livox_custom;
    ros::param::get("is_livox_custom", is_livox_custom);

    ros::Subscriber sub = nh.subscribe(cloud_topic, 1, CloudCallback);
    ros::spin();
    return 0;
}
