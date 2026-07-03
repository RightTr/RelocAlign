#include "ros_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib> 

int main(int argc, char** argv)
{
    ros_utils::init(argc, argv, "manual_reloc_pub");

    auto pub = ros_utils::advertise<ros_utils::PoseWithCovarianceStampedMsg>("reloc/manual", 1);

    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;

    if(argc >= 5)
    {
        x = std::atof(argv[1]);
        y = std::atof(argv[2]);
        z = std::atof(argv[3]);
        yaw = std::atof(argv[4]);
        ros_utils::print_info("Using input pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", x, y, z, yaw);
    }
    else
    {
        ros_utils::print_warn("Not enough arguments, using default pose (0,0,0,0)");
    }

    ros_utils::sleep(0.5); 

    ros_utils::PoseWithCovarianceStampedMsg msg;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    msg.header.stamp = ros_utils::now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    ros_utils::publish(pub, msg);
    ros_utils::print_info("Pose published to reloc/manual");

    ros_utils::spin_once();
    ros_utils::sleep(0.5);

    return 0;
}
