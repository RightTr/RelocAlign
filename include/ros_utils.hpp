#ifndef RELOCALIGN_ROS_UTILS_HPP_
#define RELOCALIGN_ROS_UTILS_HPP_

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#if defined(USE_ROS1)
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#elif defined(USE_ROS2)
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

namespace ros_utils
{
#if defined(USE_ROS1)
using TimeMsg = ros::Time;
using PoseWithCovarianceStampedMsg = geometry_msgs::PoseWithCovarianceStamped;
using PointCloud2Msg = sensor_msgs::PointCloud2;
using PointCloud2MsgConstPtr = sensor_msgs::PointCloud2ConstPtr;
using LivoxCustomMsg = livox_ros_driver2::CustomMsg;
using LivoxCustomMsgConstPtr = livox_ros_driver2::CustomMsg::ConstPtr;

template <typename T>
using Publisher = ros::Publisher;

template <typename T>
using Subscriber = ros::Subscriber;

inline void init(int argc, char **argv, const std::string &name)
{
  ros::init(argc, argv, name);
}

inline bool ok()
{
  return ros::ok();
}

inline void shutdown()
{
  ros::shutdown();
}

inline void spin_once()
{
  ros::spinOnce();
}

inline void spin()
{
  ros::spin();
}

inline void sleep(double seconds)
{
  ros::Duration(seconds).sleep();
}

inline TimeMsg now()
{
  return ros::Time::now();
}

inline ros::NodeHandle &node()
{
  static ros::NodeHandle nh;
  return nh;
}

template <typename T>
inline void get_param(const std::string &name, T &value, const T &default_value)
{
  node().param<T>(name, value, default_value);
}

template <typename T>
inline Publisher<T> advertise(const std::string &topic, uint32_t queue_size)
{
  return node().advertise<T>(topic, queue_size);
}

template <typename T, typename Callback>
inline Subscriber<T> subscribe(const std::string &topic, uint32_t queue_size, Callback callback)
{
  return node().subscribe<T>(topic, queue_size, callback);
}

template <typename PubT, typename MsgT>
inline void publish(PubT &pub, const MsgT &msg)
{
  pub.publish(msg);
}

#elif defined(USE_ROS2)
using TimeMsg = builtin_interfaces::msg::Time;
using PoseWithCovarianceStampedMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointCloud2MsgConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomMsgConstPtr = livox_ros_driver2::msg::CustomMsg::ConstSharedPtr;

template <typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

template <typename T>
using Subscriber = rclcpp::SubscriptionBase::SharedPtr;

inline rclcpp::Node::SharedPtr &node()
{
  static rclcpp::Node::SharedPtr instance;
  return instance;
}

inline void init(int argc, char **argv, const std::string &name)
{
  rclcpp::init(argc, argv);
  if (!node()) {
    node() = rclcpp::Node::make_shared(name);
  }
}

inline bool ok()
{
  return rclcpp::ok();
}

inline void shutdown()
{
  rclcpp::shutdown();
}

inline void spin_once()
{
  if (node()) {
    rclcpp::spin_some(node());
  }
}

inline void spin()
{
  if (node()) {
    rclcpp::spin(node());
  }
}

inline void sleep(double seconds)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

inline TimeMsg now()
{
  const auto stamp = node() ? node()->get_clock()->now().to_msg() : rclcpp::Clock().now().to_msg();
  return stamp;
}

template <typename T>
inline void get_param(const std::string &name, T &value, const T &default_value)
{
  if (!node()) {
    node() = rclcpp::Node::make_shared("relocalign");
  }

  if constexpr (std::is_same_v<T, float>) {
    if (!node()->has_parameter(name)) {
      node()->declare_parameter<double>(name, static_cast<double>(default_value));
    }
    value = static_cast<float>(node()->get_parameter(name).get_value<double>());
  } else {
    if (!node()->has_parameter(name)) {
      node()->declare_parameter<T>(name, default_value);
    }
    value = node()->get_parameter(name).get_value<T>();
  }
}

template <typename T>
inline Publisher<T> advertise(const std::string &topic, uint32_t queue_size)
{
  if (!node()) {
    node() = rclcpp::Node::make_shared("relocalign");
  }
  return node()->create_publisher<T>(topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));
}

template <typename T, typename Callback>
inline Subscriber<T> subscribe(const std::string &topic, uint32_t queue_size, Callback callback)
{
  if (!node()) {
    node() = rclcpp::Node::make_shared("relocalign");
  }
  return node()->create_subscription<T>(topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)), callback);
}

template <typename PubT, typename MsgT>
inline void publish(const PubT &pub, const MsgT &msg)
{
  if (pub) {
    pub->publish(msg);
  }
}

#endif

inline void print_warn(const char *fmt, ...)
{
  char msg[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
#if defined(USE_ROS1)
  ROS_WARN("%s", msg);
#elif defined(USE_ROS2)
  RCLCPP_WARN(rclcpp::get_logger("relocalign"), "%s", msg);
#endif
}

inline void print_error(const char *fmt, ...)
{
  char msg[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
#if defined(USE_ROS1)
  ROS_ERROR("%s", msg);
#elif defined(USE_ROS2)
  RCLCPP_ERROR(rclcpp::get_logger("relocalign"), "%s", msg);
#endif
}

inline void print_info(const char *fmt, ...)
{
  char msg[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
#if defined(USE_ROS1)
  ROS_INFO("%s", msg);
#elif defined(USE_ROS2)
  RCLCPP_INFO(rclcpp::get_logger("relocalign"), "%s", msg);
#endif
}

}  // namespace ros_utils

#endif
