#include "ros_utils.hpp"
#include "relocalign.hpp"
#include "read_configs.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
int frame_count;
int count = 0;
RelocAlign relocalign;
bool is_livox_custom = false;
std::string cloud_topic;
std::string config_path;
std::string map_path;

ros_utils::Subscriber<ros_utils::PointCloud2Msg> sub_livox;
ros_utils::Publisher<ros_utils::PoseWithCovarianceStampedMsg> pub_reloc;

void LivoxCallbackCustom(const ros_utils::LivoxCustomMsgConstPtr &msg);
void LivoxCallback(const ros_utils::PointCloud2MsgConstPtr &cloud_msg);
void PublishPose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q);

int main(int argc, char * argv[])
{
    ros_utils::init(argc, argv, "relocalign_pub");

    ros_utils::get_param("config_path", config_path, std::string{});
    ros_utils::get_param("is_livox_custom", is_livox_custom, false);
    ros_utils::get_param("cloud_topic", cloud_topic, std::string{});
    ros_utils::get_param("frame_count", frame_count, 1);
    ros_utils::get_param("map_path", map_path, std::string{});

    RelocAlignConfig relocalignconfig(config_path);
    relocalign = RelocAlign(relocalignconfig);

    pub_reloc = ros_utils::advertise<ros_utils::PoseWithCovarianceStampedMsg>("reloc/cloud_align", 10);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud) == -1){
        std::cout << "[RelocAlign] Couldn't read map file ......\n" << std::endl;
        return -1;
    }else{
        std::cout << "[RelocAlign] Map cloud size: " << map_cloud->size() << std::endl;
    }
    
    if(is_livox_custom){
        sub_livox = ros_utils::subscribe<ros_utils::LivoxCustomMsg>(cloud_topic, 10, LivoxCallbackCustom);
    }else{
        sub_livox = ros_utils::subscribe<ros_utils::PointCloud2Msg>(cloud_topic, 10, LivoxCallback);
    }

    while(ros_utils::ok()){
        ros_utils::spin_once();
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
        ros_utils::sleep(0.1);
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
    ros_utils::PoseWithCovarianceStampedMsg msg;

    msg.header.stamp = ros_utils::now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    ros_utils::publish(pub_reloc, msg);
}

void LivoxCallbackCustom(const ros_utils::LivoxCustomMsgConstPtr &msg){
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

void LivoxCallback(const ros_utils::PointCloud2MsgConstPtr &msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *current_cloud);
    AccumulateCloud(current_cloud);
}
