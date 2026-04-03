// Copyright (c) 2020, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "publish/publish.hpp"
#include "core/util.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace iris
{

void publishPose(
  tf2_ros::TransformBroadcaster & br,
  const Eigen::Matrix4f & T,
  const std::string & child_frame_id,
  const rclcpp::Time & stamp)
{
  Eigen::Matrix4f Tn = util::normalizePose(T);
  Eigen::Matrix3f R = Tn.topLeftCorner(3, 3);
  Eigen::Quaternionf q(R);

  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = stamp;
  ts.header.frame_id = "world";
  ts.child_frame_id = child_frame_id;
  ts.transform.translation.x = Tn(0, 3);
  ts.transform.translation.y = Tn(1, 3);
  ts.transform.translation.z = Tn(2, 3);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  br.sendTransform(ts);
}

void publishPointcloud(
  PointCloud2Pub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = "world";
  msg.header.stamp = stamp;
  publisher->publish(msg);
}

void publishImage(image_transport::Publisher & publisher, const cv::Mat & image)
{
  std_msgs::msg::Header header;
  auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  publisher.publish(*msg);
}

void publishCorrespondences(
  MarkerPub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
  const pcl::CorrespondencesPtr & correspondences,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = stamp;
  line_strip.ns = "correspondences";
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.scale.x = 0.15;
  line_strip.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_strip.color.r = 1.0;
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.0;
  line_strip.color.a = 1.0;

  for (const pcl::Correspondence & c : *correspondences) {
    const pcl::PointXYZ & point1 = source->at(c.index_query);
    const pcl::PointXYZ & point2 = target->at(c.index_match);
    geometry_msgs::msg::Point p1, p2;
    p1.x = point1.x; p1.y = point1.y; p1.z = point1.z;
    p2.x = point2.x; p2.y = point2.y; p2.z = point2.z;
    line_strip.points.push_back(p1);
    line_strip.points.push_back(p2);
  }
  publisher->publish(line_strip);
}

void publishNormal(
  MarkerArrayPub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  geometry_msgs::msg::Vector3 arrow;
  arrow.x = 0.2;
  arrow.y = 0.4;
  arrow.z = 0.5;

  for (size_t id = 0; id < cloud->size(); id++) {
    visualization_msgs::msg::Marker marker;
    const pcl::PointXYZ & p = cloud->at(id);
    const pcl::Normal & n = normals->at(id);

    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.ns = "normal";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = static_cast<int>(id);
    marker.scale = arrow;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point p1, p2;
    p1.x = p.x; p1.y = p.y; p1.z = p.z;
    p2.x = p.x + 2.0f * n.normal_x;
    p2.y = p.y + 2.0f * n.normal_y;
    p2.z = p.z + 2.0f * n.normal_z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker_array.markers.push_back(marker);
  }
  publisher->publish(marker_array);
}

void publishCovariance(
  MarkerArrayPub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  geometry_msgs::msg::Vector3 diameter;
  diameter.x = 4.0;
  diameter.y = 4.0;
  diameter.z = 1.0;

  for (size_t id = 0; id < cloud->size(); id++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.ns = "covariance";
    marker.action = visualization_msgs::msg::Marker::ADD;

    const pcl::PointXYZ & p = cloud->at(id);
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;

    const pcl::Normal & n = normals->at(id);
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
      Eigen::Vector3f(0, 0, 1),
      Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.id = static_cast<int>(id);
    marker.scale = diameter;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;
    marker_array.markers.push_back(marker);
  }
  publisher->publish(marker_array);
}

void publishPath(
  PathPub & publisher,
  const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> & trajectory,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "world";
  path.header.stamp = stamp;

  for (const Eigen::Vector3f & t : trajectory) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = t.x();
    pose_stamped.pose.position.y = t.y();
    pose_stamped.pose.position.z = t.z();
    pose_stamped.pose.orientation.w = 1.0;
    path.poses.push_back(pose_stamped);
  }
  publisher->publish(path);
}

void publishResetPointcloud(PointCloud2Pub & publisher, const rclcpp::Time & stamp)
{
  pcl::PointCloud<pcl::PointXYZ> empty;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(empty, msg);
  msg.header.frame_id = "world";
  msg.header.stamp = stamp;
  publisher->publish(msg);
}

void publishResetCorrespondences(MarkerPub & publisher, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = stamp;
  line_strip.ns = "correspondences";
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.scale.x = 0.3;
  line_strip.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_strip.color.r = 1.0;
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.0;
  line_strip.color.a = 1.0;
  publisher->publish(line_strip);
}

}  // namespace iris
