#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib> 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_reloc_pub");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("reloc/manual", 1);

    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;

    if(argc >= 5)
    {
        x = std::atof(argv[1]);
        y = std::atof(argv[2]);
        z = std::atof(argv[3]);
        yaw = std::atof(argv[4]);
        ROS_INFO("Using input pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", x, y, z, yaw);
    }
    else
    {
        ROS_WARN("Not enough arguments, using default pose (0,0,0,0)");
    }

    ros::Duration(0.5).sleep(); 

    geometry_msgs::PoseWithCovarianceStamped msg;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    pub.publish(msg);
    ROS_INFO("Pose published to reloc/manual");

    ros::spinOnce();
    ros::Duration(0.5).sleep();

    return 0;
}
