#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <livox_ros_driver/CustomMsg.h>

void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);  
    pcl::io::savePCDFileBinary("frame.pcd", cloud);
    ROS_INFO("Saved one frame point cloud with %zu points", cloud.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collect_node");
    ros::NodeHandle nh;

    std::string pcd_path;
    ros::param::get("pcd_path", pcd_path);

    std::string cloud_topic;
    ros::param::get("cloud_topic", pcd_path);

    bool is_livox_custom;
    ros::param::get("is_livox_custom", is_livox_custom);

    ros::Subscriber sub = nh.subscribe("/cloud_pcd", 1, CloudCallback);
    ros::spin();
    return 0;
}
