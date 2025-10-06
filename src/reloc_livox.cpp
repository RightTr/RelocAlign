#include <ros/ros.h>
#include "relocalign.hpp"
#include "read_configs.hpp"
#include <pcl/point_types.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
int frame_count;
int count = 0;
RelocAlign relocalign;
bool is_livox_custom = false;
std::string cloud_topic;
std::string config_path;
std::string map_path;

ros::Subscriber sub_livox;
ros::Publisher pub_reloc;

void LivoxCallbackCustom(const livox_ros_driver2::CustomMsg::ConstPtr& msg);
void LivoxCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void PublishPose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q);

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "relocalign_pub");
    ros::NodeHandle nh;

    ros::param::get("config_path", config_path);
    ros::param::get("is_livox_custom", is_livox_custom);
    ros::param::get("cloud_topic", cloud_topic);
    ros::param::get("frame_count", frame_count);
    ros::param::get("map_path", map_path);

    RelocAlignConfig relocalignconfig(config_path);
    relocalign = RelocAlign(relocalignconfig);

    pub_reloc = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("reloc/cloud_align", 10);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud) == -1){
        std::cout << "[RelocAlign] Couldn't read map file ......\n" << std::endl;
        return -1;
    }else{
        std::cout << "[RelocAlign] Map cloud size: " << map_cloud->size() << std::endl;
    }
    
    if(is_livox_custom){
        sub_livox = nh.subscribe<livox_ros_driver2::CustomMsg>(cloud_topic, 10, LivoxCallbackCustom);
    }else{
        sub_livox = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 10, LivoxCallback);
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(10);
    while(ros::ok()){
        if(count == frame_count-1){
            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            source_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*accumulated_cloud));
            target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*map_cloud));

            source_cloud->erase(
                std::remove_if(source_cloud->begin(), source_cloud->end(), [=](const pcl::PointXYZ& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
                source_cloud->end());
            target_cloud->erase(
                std::remove_if(target_cloud->begin(), target_cloud->end(), [=](const pcl::PointXYZ& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
                target_cloud->end());
            
            float voxelgrid_leaf = relocalignconfig.voxelgrid_leaf;
            pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(voxelgrid_leaf, voxelgrid_leaf, voxelgrid_leaf);

            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            vg.setInputCloud(source_cloud);
            vg.filter(*source_cloud_filtered);

            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            vg.setInputCloud(target_cloud);
            vg.filter(*target_cloud_filtered);

            relocalign.SourceCloudInput(source_cloud_filtered);
            relocalign.TargetCloudInput(target_cloud_filtered);

            relocalign.Align();

            Eigen::Vector3d t;
            Eigen::Quaterniond q;
            relocalign.GetTransform(t, q);
            PublishPose(t, q);

            std::cout << "[RelocAlign] Estimated Transformation: \n";
            std::cout << "[RelocAlign] Translation: \n" << t.transpose() << std::endl;
            std::cout << "[RelocAlign] Rotation (quaternion): \n" << q.coeffs().transpose() << std::endl;

            count = 0;
            accumulated_cloud->clear();
        }
    }
    return 0;
}

void AccumulateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud){
    if(input_cloud->empty()){
        std::cerr << "[RelocAlign Error] Received empty pointcloud!" << std::endl;
        return;
    }

    if(count == 0) 
    {
        *accumulated_cloud = *input_cloud;
    } 
    else if(count < frame_count)
    {
        *accumulated_cloud += *input_cloud;
    }
    else
    {
        return;
    }
    count++;

    // std::cout << "[RelocAlign INFO] Frame count: " << frame_count 
    //           << ", Accumulated cloud size: " << accumulated_cloud->size() << std::endl;
}

void PublishPose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q){
    geometry_msgs::PoseWithCovarianceStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    pub_reloc.publish(msg);
}

void LivoxCallbackCustom(const livox_ros_driver2::CustomMsg::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(auto& p : msg->points)
    {
        pcl::PointXYZ pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        current_cloud->push_back(pt);
    }
    AccumulateCloud(current_cloud);
}

void LivoxCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *current_cloud);
    AccumulateCloud(current_cloud);
}