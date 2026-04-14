#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// NOTE: Callbacks and timer run on rclcpp::spin's single thread, so no mutex
// is needed. If a MultiThreadedExecutor is used, subscribed_image_ and
// subscribed_stamp_ must be protected with a std::mutex.

inline void publishPose(
  tf2_ros::TransformBroadcaster & broadcaster,
  const rclcpp::Node & node,
  const Eigen::Matrix4f & T,
  const std::string & child_frame_id)
{
  Eigen::Matrix3f R = T.topLeftCorner(3, 3);
  Eigen::Quaternionf q(R);
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = node.now();
  ts.header.frame_id = "world";
  ts.child_frame_id = child_frame_id;
  ts.transform.translation.x = T(0, 3);
  ts.transform.translation.y = T(1, 3);
  ts.transform.translation.z = T(2, 3);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  broadcaster.sendTransform(ts);
}
