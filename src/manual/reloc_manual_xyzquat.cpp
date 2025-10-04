#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cstdlib> 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_reloc_pub");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("reloc/manual", 10);

    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;

    if(argc >= 8)
    {
        x = std::atof(argv[1]);
        y = std::atof(argv[2]);
        z = std::atof(argv[3]);
        qx = std::atof(argv[4]);
        qy = std::atof(argv[5]);
        qz = std::atof(argv[6]);
        qw = std::atof(argv[7]);
        ROS_INFO("Using input pose: x=%.2f, y=%.2f, z=%.2f, \
            qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f", x, y, z, qx, qy, qz, qw);
    }
    else
    {
        ROS_WARN("Not enough arguments, using default pose (0,0,0,0,0,0,0)");
    }

    ros::Duration(0.5).sleep();

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;

    msg.pose.pose.orientation.x = qx;
    msg.pose.pose.orientation.y = qy;
    msg.pose.pose.orientation.z = qz;
    msg.pose.pose.orientation.w = qw;

    pub.publish(msg);
    ROS_INFO("Pose published to reloc/manual");

    ros::spinOnce();
    ros::Duration(0.5).sleep();


    return 0;
}
